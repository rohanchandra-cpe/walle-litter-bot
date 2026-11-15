# walle-litter-bot

All the code is in [`walle-pilot/`](walle-pilot/README.md). The containers are defined in
`docker-compose.yml`; from this folder:

```bash
make up      # start the pilot + micro-ROS Agent containers
make shell   # open a shell in the pilot container (run again for more shells)
make down    # stop them
```

Run `make` to list the other shortcuts.

## Gazebo test environments

[`walle-sim/`](walle-sim/README.md) runs the same pilot code against a simulated WALL-E in
Gazebo, with one world per research question (RQ1 lane, RQ2 protractor, RQ3 ranging room)
plus an inspection room for agent testing. Try new firmware logic, tools or agents there
first:

```bash
make sim-up                         # build and start the simulator container
make sim-launch WORLD=motion_area   # Gazebo at http://localhost:6080/vnc.html
make sim-shell                      # then: python3 /workspace/walle-sim/harness/sim_trials.py check
```

It uses its own Compose project and ROS domain, so it can run next to `make up` without
touching the robot.
