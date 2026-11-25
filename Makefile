# Shortcuts for the WALL-E containers. Run `make` (or `make help`) to list them.

COMPOSE := docker compose
SVC     := pilot

.DEFAULT_GOAL := help
.PHONY: help up build down restart shell launch fake logs agent-logs ps \
        sim-up sim-build sim-down sim-shell sim-launch sim-check sim-worlds \
        sim-park sim-park-check sim-route-check sim-bin-check sim-obstacle-check sim-trip-check sim-look-check sim-agent-check \
        detect-eval

# Gazebo simulator (walle-sim/README.md): a separate Compose project and ROS domain
SIM_COMPOSE := docker compose -f docker-compose.sim.yml
WORLD   ?= motion_area
PROFILE ?= pilot_firmware
GUI     ?= true
RUN     ?= default
# world park only (walle-app/config/scenarios/): "scenario" keeps the file's value
SCENARIO ?= default
BINS     ?= scenario
OBSTACLE ?= scenario
SEED     ?= scenario
LOOK_AT  ?=
LAPS     ?= 1
DWELL    ?=
COAST    ?=
MISPLACE ?=
HIDE_CUT ?=
BLACKOUT ?=
RESTART  ?=
JUMP     ?=
RUNS     ?=
CASES    ?=
MODEL    ?=

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
	@echo "make detect-eval  score the detector on the labeled photo set (MODEL=yolov8m-worldv2.pt)"
	@echo ""
	@echo "Gazebo simulator (walle-sim/README.md):"
	@echo "make sim-up      start the simulator container (builds if needed)"
	@echo "make sim-shell   open a bash shell in it"
	@echo "make sim-launch  start Gazebo + simulated Pico + odometry/EKF"
	@echo "                 WORLD=motion_area|rq1_lane|rq2_protractor|rq3_ranging|rq3_ranging_dim|inspection_room|park"
	@echo "                 park: SCENARIO=default|mixed|obstacle_only  BINS=0-4  OBSTACLE=none|random|1-4  SEED=n"
	@echo "                 PROFILE=pilot_firmware|ideal|pid_firmware|stress|ultrasonic_dead  GUI=true|false"
	@echo "make sim-check   is everything publishing? (RUN=name picks the data folder)"
	@echo "make sim-park    print a park layout (same SCENARIO/BINS/OBSTACLE/SEED) without launching"
	@echo "make sim-park-check  world park: every bin/item/obstacle in place? LOOK_AT=\"bin_2 obstacle\" saves photos"
	@echo "make sim-route-check  world park: drive the route (LAPS=1 DWELL=s COAST=s MISPLACE=deg HIDE_CUT=1 BLACKOUT="every_s for_s"), PASS if it ends within 5 cm"
	@echo "make sim-bin-check  world park, Phase 4: servo sweep + route-check with every bin check judged against the world (same options)"
	@echo "make sim-obstacle-check  Phase 5: launches each obstacle case headless (no launch may be running), PASS if 9 of 10 pass (RUNS=n CASES=\"near lying\")"
	@echo "                  + Phase 6 clockwise trip cases (cw_*), all must pass (CW=0 skips them, CW=only runs only them)"
	@echo "make sim-trip-check BIN=n  world park, Phase 6: the follower's --trip-bin n (to bin n and back), PASS if it ends within 5 cm and the bin is judged right"
	@echo "make sim-look-check  world park, Phase 6: the follower's --look-only from the start pose, finds the side-1 obstacle (launch OBSTACLE=1)"
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
	$(SIM_COMPOSE) exec sim bash -ic "ros2 launch /workspace/walle-sim/launch/sim.launch.py world:=$(WORLD) profile:=$(PROFILE) gui:=$(GUI) scenario:=$(SCENARIO) bins:=$(BINS) obstacle:=$(OBSTACLE) park_seed:=$(SEED)"

sim-check:
	$(SIM_COMPOSE) exec sim bash -ic "python3 /workspace/walle-sim/harness/sim_trials.py --run $(RUN) check"

sim-worlds:
	$(SIM_COMPOSE) exec sim bash -ic "python3 /workspace/walle-sim/worlds/generate_worlds.py"

sim-park:
	$(SIM_COMPOSE) exec sim bash -ic "python3 /workspace/walle-app/park.py --scenario $(SCENARIO) $(if $(filter-out scenario,$(BINS)),--bins $(BINS)) $(if $(filter-out scenario,$(OBSTACLE)),--obstacle $(OBSTACLE)) $(if $(filter-out scenario,$(SEED)),--seed $(SEED))"

sim-park-check:
	$(SIM_COMPOSE) exec sim bash -ic "python3 /workspace/walle-sim/harness/sim_trials.py --run $(RUN) park-check $(if $(LOOK_AT),--look-at $(LOOK_AT))"

sim-route-check:
	$(SIM_COMPOSE) exec sim bash -ic "python3 /workspace/walle-sim/harness/sim_trials.py --run $(RUN) route-check --laps $(LAPS) $(if $(DWELL),--dwell $(DWELL)) $(if $(COAST),--coast-time $(COAST)) $(if $(MISPLACE),--misplace-deg $(MISPLACE)) $(if $(HIDE_CUT),--hide-cut-bins) $(if $(BLACKOUT),--blackout $(BLACKOUT)) $(if $(RESTART),--restart $(RESTART)) $(if $(JUMP),--jump $(JUMP))"

sim-bin-check:
	$(SIM_COMPOSE) exec sim bash -ic "python3 /workspace/walle-sim/harness/sim_trials.py --run $(RUN) bin-check --laps $(LAPS) $(if $(DWELL),--dwell $(DWELL)) $(if $(COAST),--coast-time $(COAST)) $(if $(MISPLACE),--misplace-deg $(MISPLACE)) $(if $(HIDE_CUT),--hide-cut-bins) $(if $(BLACKOUT),--blackout $(BLACKOUT)) $(if $(RESTART),--restart $(RESTART)) $(if $(JUMP),--jump $(JUMP))"

# Phase 5: obstacle cases, one headless world each (walle-sim/harness/obstacle_trials.py)
sim-obstacle-check:
	$(SIM_COMPOSE) exec sim bash -ic "python3 /workspace/walle-sim/harness/obstacle_trials.py $(if $(RUNS),--runs $(RUNS)) $(if $(CASES),--only $(CASES)) $(if $(filter 0,$(CW)),--no-cw) $(if $(filter only,$(CW)),--cw-only)"

# Phase 6: the follower's partial route (--trip-bin) and look-only mode, against Gazebo
sim-trip-check:
	$(SIM_COMPOSE) exec sim bash -ic "python3 /workspace/walle-sim/harness/sim_trials.py --run $(RUN) trip-check --bin $(BIN) $(if $(DWELL),--dwell $(DWELL)) $(if $(COAST),--coast-time $(COAST))"

sim-look-check:
	$(SIM_COMPOSE) exec sim bash -ic "python3 /workspace/walle-sim/harness/sim_trials.py --run $(RUN) look-check"

# Phase 6: the agent's acceptance prompts (walle-sim/harness/agent_trials.py); world cases
# launch their own headless worlds (stop any launch first). ONLY="history route bin1 look"
sim-agent-check:
	$(SIM_COMPOSE) exec sim bash -ic "python3 /workspace/walle-sim/harness/agent_trials.py $(if $(ONLY),--only $(ONLY)) $(if $(REPEATS),--repeats $(REPEATS))"

# Phase 3: score YOLO-World on the labeled ESP32-CAM photos (walle-app/data/photos/phase3/)
detect-eval:
	$(COMPOSE) exec $(SVC) bash -ic "cd /workspace && python3 walle-app/detect/eval_photos.py $(if $(MODEL),--model $(MODEL))"
