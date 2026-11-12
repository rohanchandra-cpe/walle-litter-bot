"""Models of the Pico's drive firmware: /pico/cmd_vel in, motor duty (or wheel speed) out.

nodes/sim_pico_bridge.py calls one of these at 50 Hz, the firmware's loop rate. The
bridge owns everything every firmware shares: the 500 ms command watchdog, the drive
status, the encoders and the motor plant.

To test a new firmware idea before writing it in C:
  1. Subclass FirmwareModel below (mirror the C logic as literally as you can).
  2. Register it in MODELS and add a profile to config/sim_profiles.yaml.
  3. Launch with profile:=<your profile> and run walle-sim/harness/sim_trials.py.
"""

import math


def clamp(value, limit):
    return max(-limit, min(limit, value))


class FirmwareModel:
    """Base class. `outputs_speed`: True if update() returns wheel speeds (m/s) that
    reach the wheels exactly; False if it returns motor duties (-1..1) for the plant."""

    outputs_speed = False

    def __init__(self, params):
        self.p = params
        self.linear = 0.0
        self.angular = 0.0

    def command(self, linear, angular):
        """A /pico/cmd_vel message (drive_control_command)."""
        self.linear = clamp(linear, self.p["max_linear_mps"])
        self.angular = clamp(angular, self.p["max_angular_rps"])

    def stop(self):
        """Watchdog or explicit stop (drive_control_stop)."""
        self.linear = self.angular = 0.0

    def wheel_targets(self):
        """Differential drive with the FIRMWARE's wheel separation constant."""
        half = self.angular * self.p["wheel_separation_m"] / 2.0
        return self.linear - half, self.linear + half

    def update(self, dt, counts):
        """Called every firmware loop. counts: (left, right) encoder totals.
        Returns (left, right) duties, or speeds if outputs_speed."""
        raise NotImplementedError

    def driving(self):
        return self.linear != 0.0 or self.angular != 0.0


class IdealFirmware(FirmwareModel):
    """The commanded twist reaches the wheels exactly: no PWM map, no motor."""

    outputs_speed = True

    def update(self, dt, counts):
        return self.wheel_targets()


class OpenLoopPwmFirmware(FirmwareModel):
    """firmware/pico_pilot/drive_control.c, line for line."""

    def wheel_duty(self, wheel_speed):
        fraction = clamp(wheel_speed / self.p["max_wheel_speed_mps"], 1.0)
        if -0.01 < fraction < 0.01:
            return 0.0
        duty = self.p["min_duty"] + (1.0 - self.p["min_duty"]) * abs(fraction)
        return -duty if fraction < 0.0 else duty

    def update(self, dt, counts):
        left, right = self.wheel_targets()
        return self.wheel_duty(left), self.wheel_duty(right)

    def driving(self):
        left, right = self.wheel_targets()
        return self.wheel_duty(left) != 0.0 or self.wheel_duty(right) != 0.0


class PidFirmware(OpenLoopPwmFirmware):
    """A closed-loop candidate: the open-loop duty as feedforward, plus a PI correction
    on each wheel's speed measured from its encoder (what the Pico can see)."""

    def __init__(self, params):
        super().__init__(params)
        self.last_counts = None
        self.integral = [0.0, 0.0]
        self.speed = [0.0, 0.0]
        self.m_per_count = math.pi * params["wheel_diameter_m"] / params["counts_per_rev"]

    def stop(self):
        super().stop()
        self.integral = [0.0, 0.0]

    def update(self, dt, counts):
        if self.last_counts is not None and dt > 0:
            for i in (0, 1):
                measured = (counts[i] - self.last_counts[i]) * self.m_per_count / dt
                self.speed[i] += 0.5 * (measured - self.speed[i])   # light smoothing
        self.last_counts = counts
        duties = []
        for i, target in enumerate(self.wheel_targets()):
            if target == 0.0:
                self.integral[i] = 0.0
                duties.append(0.0)
                continue
            error = target - self.speed[i]
            self.integral[i] = clamp(self.integral[i] + error * dt, 0.2)
            duty = self.wheel_duty(target) + self.p["kp"] * error + self.p["ki"] * self.integral[i]
            duties.append(clamp(duty, 1.0))
        return tuple(duties)


MODELS = {"ideal": IdealFirmware, "open_loop_pwm": OpenLoopPwmFirmware, "pid": PidFirmware}


def make_firmware(name, params):
    if name not in MODELS:
        raise ValueError(f"Unknown firmware model '{name}'. Choose one of: {', '.join(MODELS)}")
    return MODELS[name](params)


class MotorPlant:
    """TT gear motor + TB6612: duty -> wheel surface speed, with a first-order lag."""

    def __init__(self, motor):
        self.m = motor
        self.speed = [0.0, 0.0]

    def steady_speed(self, duty, gain):
        if abs(duty) < self.m["stall_duty"]:
            return 0.0
        return duty * self.m["full_speed_mps"] * gain

    def update(self, dt, duties=None, speeds=None):
        gains = (self.m["left_gain"], self.m["right_gain"])
        if speeds is not None:   # ideal firmware: the target speed is the wheel speed
            targets = speeds
        else:
            targets = [self.steady_speed(d, g) for d, g in zip(duties, gains)]
        tau = self.m["time_constant_s"]
        alpha = 1.0 if tau <= 0 else min(1.0, dt / tau)
        for i, target in enumerate(targets):
            self.speed[i] += alpha * (target - self.speed[i])
            if target == 0.0 and abs(self.speed[i]) < 1e-3:
                self.speed[i] = 0.0
        return tuple(self.speed)
