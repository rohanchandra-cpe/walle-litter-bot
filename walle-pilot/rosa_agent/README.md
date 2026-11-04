# ROSA agent

The ROSA ReAct agent and custom tools described in the paper, copied unchanged from
Module 8. Fixed scripts run the experiments, so nothing in this folder is used by the
pilot or the main study.

| Agent | Tools |
|---|---|
| `odometry/odom_agent.py` | `get_pose`, `drive_distance`, `turn`, `go_to`, `stop_robot` |
| `vision/vision_agent.py` | `look`, `describe_scene`, `search_for`, `approach`, `drive_forward_if_clear`, plus `turn`, `get_pose`, `stop_robot` |
| `depth/depth_agent.py` | `measure_distance`, `approach_to`, plus `turn`, `get_pose`, `stop_robot` |

The agents reach Ollama at `OLLAMA_URL` (default `http://host.docker.internal:11434`).
`vision_agent.py` plans with `qwen3.5:4b`; the other two default to `llama3.2`, set with
`OLLAMA_MODEL`. The vision node uses `VISION_MODEL` (default `gemma4`).

The docstrings and launch files keep their Module 8 paths, which expect this folder's
contents at `/workspace`. Run `launch/pilot.launch.py` first so the agents have
`/odometry/filtered`. The vision and depth agents also need their launch file running.
Every tool that moves the robot asks a person for approval first.
