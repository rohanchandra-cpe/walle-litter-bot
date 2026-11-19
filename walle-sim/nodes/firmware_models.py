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


class PidFirmware(FirmwareModel):
    """firmware/pico_pilot/drive_control.c since 2026-11-18 (closed-loop wheel speed),
    line for line: feed-forward + PI on each wheel's encoder speed, a setpoint ramp, an
    immediate stop on a zero target, anti-windup, and the MIN_DUTY dead-zone skip. The
    C runs it every CONTROL_PERIOD_MS (20 ms); the bridge calls update() at 50 Hz."""

    def __init__(self, params):
        super().__init__(params)
        self.last_counts = None
        self.target = [0.0, 0.0]
        self.setpoint = [0.0, 0.0]
        self.measured = [0.0, 0.0]
        self.integral = [0.0, 0.0]
        self.effort = [0.0, 0.0]
        self.m_per_count = math.pi * params["wheel_diameter_m"] / params["counts_per_rev"]

    def command(self, linear, angular):
        super().command(linear, angular)
        self.target = list(self.wheel_targets())
        if self.target == [0.0, 0.0]:
            self.stop()

    def stop(self):
        super().stop()
        self.target = [0.0, 0.0]
        self.setpoint = [0.0, 0.0]
        self.integral = [0.0, 0.0]
        self.effort = [0.0, 0.0]

    def effort_to_duty(self, effort):
        if abs(effort) < 0.01:
            return 0.0
        duty = self.p["pid_min_duty"] + (1.0 - self.p["pid_min_duty"]) * abs(effort)
        return -duty if effort < 0.0 else duty

    def update(self, dt, counts):
        if self.last_counts is None or dt <= 0:
            self.last_counts = counts
            return 0.0, 0.0
        p = self.p
        for i in (0, 1):
            raw = (counts[i] - self.last_counts[i]) * self.m_per_count / dt
            self.measured[i] = p["speed_filter"] * self.measured[i] + (1 - p["speed_filter"]) * raw
            if self.target[i] == 0.0:
                self.setpoint[i] = self.integral[i] = self.effort[i] = 0.0
                continue
            self.setpoint[i] += clamp(self.target[i] - self.setpoint[i], p["max_accel_mps2"] * dt)
            error = self.setpoint[i] - self.measured[i]
            integral = self.integral[i] + error * dt
            u = self.setpoint[i] / p["max_wheel_speed_mps"] + p["kp"] * error + p["ki"] * integral
            if u > 1.0:
                u = 1.0
            elif u < -1.0:
                u = -1.0
            else:
                self.integral[i] = integral
            self.effort[i] = u
        self.last_counts = counts
        return self.effort_to_duty(self.effort[0]), self.effort_to_duty(self.effort[1])

    def driving(self):
        return self.target != [0.0, 0.0]


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

    def target_speed(self, i, duty, gain):
        """Stiction: a STOPPED wheel needs breakaway_duty to start; once it turns, it keeps
        turning down to stall_duty. With breakaway > stall, slow speeds stick and lurch
        (measured on the robot 2026-11-18: slow turns jumped between 0 and 3x the target)."""
        if abs(self.speed[i]) < 0.005 and abs(duty) < self.m.get("breakaway_duty", 0.0):
            return 0.0
        return self.steady_speed(duty, gain)

    def update(self, dt, duties=None, speeds=None):
        gains = (self.m["left_gain"], self.m["right_gain"])
        if speeds is not None:   # ideal firmware: the target speed is the wheel speed
            targets = speeds
        else:
            targets = [self.target_speed(i, d, g) for i, (d, g) in enumerate(zip(duties, gains))]
        tau = self.m["time_constant_s"]
        alpha = 1.0 if tau <= 0 else min(1.0, dt / tau)
        for i, target in enumerate(targets):
            self.speed[i] += alpha * (target - self.speed[i])
            if target == 0.0 and abs(self.speed[i]) < 1e-3:
                self.speed[i] = 0.0
        return tuple(self.speed)
