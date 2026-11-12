# Shortcuts for the WALL-E containers. Run `make` (or `make help`) to list them.

COMPOSE := docker compose
SVC     := pilot

.DEFAULT_GOAL := help
.PHONY: help up build down restart shell launch fake logs agent-logs ps

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
