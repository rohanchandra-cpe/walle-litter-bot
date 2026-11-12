# ROSA agent

The ROSA ReAct agent and custom tools described in the paper, copied from Module 8 with
paths updated for this repository, plus the range agent. Fixed scripts run the
experiments, so nothing in this folder is used by the pilot or the main study.

| Agent | Tools |
|---|---|
| `odometry/odom_agent.py` | `get_pose`, `drive_distance`, `turn`, `go_to`, `stop_robot` |
| `vision/vision_agent.py` | `look`, `describe_scene`, `search_for`, `approach`, `drive_forward_if_clear`, plus `turn`, `get_pose`, `stop_robot` |
| `depth/depth_agent.py` | `measure_distance`, `approach_to`, plus `turn`, `get_pose`, `stop_robot` |
| `range/range_agent.py` | `identify_and_range`, `measure_ultrasonic`, plus `turn`, `get_pose`, `stop_robot` |

The agents reach Ollama at `OLLAMA_URL` (default `http://host.docker.internal:11434`).
`vision_agent.py` plans with `qwen3.5:4b`, and `range_agent.py` defaults to it. The other
two default to `llama3.2`. Set `OLLAMA_MODEL` to change any but `vision_agent.py`. The vision node uses `VISION_MODEL` (default `gemma4`).

Run everything inside the container. From `/workspace/walle-pilot`, start
`launch/pilot.launch.py` first so the agents have `/odometry/filtered`. The vision, depth,
and range agents also need a launch file from `rosa_agent/launch/` running
(`vision.launch.py` for the range agent).

`identify_and_range` only measures a target in the center third of the photo, because the
HC-SR04 reports the nearest surface straight ahead. Its readings go to
`range/results/range_log.csv`. It uses the `ranging` settings in `config/pilot.yaml`.

Every tool that moves the robot asks a person for approval first.
