# Shortcuts for the WALL-E containers. Run `make` (or `make help`) to list them.

COMPOSE := docker compose
SVC     := pilot

.DEFAULT_GOAL := help
.PHONY: help up build down restart shell launch fake logs agent-logs ps \
        sim-up sim-build sim-down sim-shell sim-launch sim-check sim-worlds

# Gazebo simulator (walle-sim/README.md): a separate Compose project and ROS domain
SIM_COMPOSE := docker compose -f docker-compose.sim.yml
WORLD   ?= motion_area
PROFILE ?= pilot_firmware
GUI     ?= true
RUN     ?= default

help:
	@echo "make up          start the pilot + micro-ROS Agent containers (builds if needed)"
	@echo "make build       rebuild the image (after changing the Dockerfile)"
	@echo "make down        stop and remove the containers"
	@echo "make restart     down, then up"
	@echo "make shell       open a bash shell in the pilot container (one per terminal)"
	@echo "make launch      run launch/pilot.launch.py in the pilot container"
	@echo "make fake        same, with the simulated robot (fake:=true)"
	@echo "make agent-logs  follow the micro-ROS Agent's output (is the Pico connected?)"
	@echo "make ps          show container status"
	@echo ""
	@echo "Gazebo simulator (walle-sim/README.md):"
	@echo "make sim-up      start the simulator container (builds if needed)"
	@echo "make sim-shell   open a bash shell in it"
	@echo "make sim-launch  start Gazebo + simulated Pico + odometry/EKF"
	@echo "                 WORLD=motion_area|rq1_lane|rq2_protractor|rq3_ranging|rq3_ranging_dim|inspection_room"
	@echo "                 PROFILE=pilot_firmware|ideal|pid_firmware|stress|ultrasonic_dead  GUI=true|false"
	@echo "make sim-check   is everything publishing? (RUN=name picks the data folder)"
	@echo "make sim-worlds  regenerate walle-sim/worlds/*.sdf after editing generate_worlds.py"
	@echo "make sim-down    stop the simulator"
	@echo "Desktop: http://localhost:6080/vnc.html"

up:
	$(COMPOSE) up -d --build

build:
	$(COMPOSE) build

down:
	$(COMPOSE) down

restart: down up

shell:
	$(COMPOSE) exec $(SVC) bash

launch:
	$(COMPOSE) exec $(SVC) bash -ic "ros2 launch launch/pilot.launch.py"

fake:
	$(COMPOSE) exec $(SVC) bash -ic "ros2 launch launch/pilot.launch.py fake:=true"

agent-logs:
	$(COMPOSE) logs -f micro-ros-agent

logs: agent-logs

ps:
	$(COMPOSE) ps

# ===== Gazebo simulator =====
sim-up:
	$(SIM_COMPOSE) up -d --build

sim-build:
	$(SIM_COMPOSE) build

sim-down:
	$(SIM_COMPOSE) down

sim-shell:
	$(SIM_COMPOSE) exec sim bash

sim-launch:
	$(SIM_COMPOSE) exec sim bash -ic "ros2 launch /workspace/walle-sim/launch/sim.launch.py world:=$(WORLD) profile:=$(PROFILE) gui:=$(GUI)"

sim-check:
	$(SIM_COMPOSE) exec sim bash -ic "python3 /workspace/walle-sim/harness/sim_trials.py --run $(RUN) check"

sim-worlds:
	$(SIM_COMPOSE) exec sim bash -ic "python3 /workspace/walle-sim/worlds/generate_worlds.py"
