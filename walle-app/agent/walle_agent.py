"""WALL-E ROSA agent (ROADMAP Phase 6): run the park inspection and answer questions about it.

Run inside the container while the robot side is up (robot: `make launch`; Gazebo:
`make sim-launch WORLD=park ...`), the robot on corner 1 facing corner 2:

    python3 /workspace/walle-app/agent/walle_agent.py                  # store: auto
    python3 /workspace/walle-app/agent/walle_agent.py --no-motion      # history questions only
    python3 /workspace/walle-app/agent/walle_agent.py --once "Which bins overflow most often?"

Every motion asks you first (one approval per run). PRESS CTRL+C TO STOP THE ROBOT: it
reaches the route follower, which stops the robot and saves the run. The Pico also stops by
itself 500 ms after the last command.

The model plans with WALL-E's tools only (agent/walle_tools.py). ROSA's own ~50 tools are
left out: one of them, ros2_topic_pub, could drive the robot around the follower's guards.
Settings: walle-app/config/agent.yaml. The LLM runs on Ollama on the host.
"""

import argparse
import json
import os
import sys
import time

AGENT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, AGENT_DIR)
import walle_tools  # noqa: E402

PERSONA = (
    "You are WALL-E, a small two-wheel inspection rover in a model park. You drive a fixed "
    "route, check the trash and recycling bins for overflow, find obstacles on the route and "
    "drive around them, and remember everything you see in a memory store.")

ENVIRONMENT = (
    "The route is a 1 m square with corners 1, 2, 3 and 4, driven counter-clockwise: "
    "1 -> 2 -> 3 -> 4 -> 1. You start on corner 1 facing corner 2. Side 1 runs from corner 1 "
    "to corner 2, side 2 from corner 2 to 3, side 3 from corner 3 to 4, side 4 from corner 4 "
    "to 1. Bin N stands just outside corner N. Bins 1 and 3 are trash bins; bins 2 and 4 are "
    "recycling bins. The bins are opaque: you can see only what is above the rim and on the "
    "ground around a bin, never how full it is inside.")

CAPABILITIES = (
    "Motion (each asks a person first): run_route(check_bins) drives one full lap, looks for "
    "obstacles on every side and checks the bins; check_bin(bin) drives to one bin, checks "
    "it and comes back; look_ahead() takes one photo of the floor ahead without driving; "
    "stop_robot() stops. Memory (no motion): most_seen_objects(place), "
    "bin_overflow_history(), recycling_report(), report_obstacles(), last_bin_check(bin), "
    "recent_runs(), recall_memory(question).")

INSTRUCTIONS = (
    "Choose the tool from the request. "
    "'Run your route' or 'check the path for obstructions': call run_route. "
    "'Check bin N': call check_bin with that number. "
    "'Is anything in front of you' or 'look ahead': call look_ahead. "
    "Questions about the past or about what you know ('historically', 'most', 'often', "
    "'generally', 'usually', 'have you seen'): use the memory tools only and never move. "
    "'What have you seen the most': most_seen_objects. 'Which bins overflow': "
    "bin_overflow_history. 'Are people recycling correctly': recycling_report. 'Where are "
    "the obstacles right now': report_obstacles, and say how many hours ago each was last "
    "seen. "
    "Call each tool once; after a motion tool has run, answer from its result and do not "
    "run it again. "
    "Answer only from what the tools returned. Use their numbers and labels exactly; never "
    "invent, estimate or round a number, and never add an object the tools did not list. "
    "Mention the limits the tool gave in one short sentence. "
    "For 'how much is it overflowing', give the overflow level (0 to 3) and the number of "
    "items on top; there is no fill level. "
    "If a tool returns an error, or the person declined, say so plainly. Keep answers short.")


def build_agent(cfg, model=None, verbose=True, tools=None):
    from langchain_ollama import ChatOllama
    from rosa import ROSA, RobotSystemPrompts

    class WalleROSA(ROSA):
        """ROSA with only the tools given: no built-in ROS, calculation or log tools."""

        def _get_tools(self, ros_version, packages, tools, blacklist):
            class Only:
                def __init__(self, t):
                    self.t = list(t or [])

                def get_tools(self):
                    return self.t
            return Only(tools)

    llm_cfg = cfg["llm"]
    kwargs = {}
    if llm_cfg.get("reasoning") is not None:
        kwargs["reasoning"] = bool(llm_cfg["reasoning"])
    llm = ChatOllama(model=model or os.environ.get("OLLAMA_MODEL") or llm_cfg["model"],
                     temperature=llm_cfg["temperature"], num_ctx=llm_cfg["num_ctx"],
                     base_url=os.environ.get("OLLAMA_URL") or llm_cfg["url"], **kwargs)
    prompts = RobotSystemPrompts(embodiment_and_persona=PERSONA,
                                 about_your_environment=ENVIRONMENT,
                                 about_your_capabilities=CAPABILITIES,
                                 critical_instructions=INSTRUCTIONS)
    return WalleROSA(ros_version=2, llm=llm, tools=tools or walle_tools.TOOLS, prompts=prompts,
                     verbose=verbose, streaming=False, accumulate_chat_history=False,
                     max_iterations=int(cfg["max_iterations"]))


STORE_NAMES = {"robot": "the robot's memory store", "sim": "the simulator's memory store"}


def respond(agent, prompt):
    """The agent's answer to one prompt, plus a source line added by code (not left to the
    model) whenever a memory tool answered: which store, how many runs, the last run."""
    walle_tools.CALLS.clear()
    answer = agent.invoke(prompt)
    sources = []
    for c in walle_tools.CALLS:
        try:
            info = json.loads(c["output"]).get("from")
        except (ValueError, AttributeError):
            info = None
        if info and info not in sources:
            sources.append(info)
    for s in sources:
        answer += (f"\n(Source: {STORE_NAMES.get(s['store'], s['store'])}, {s['runs']} runs, "
                   f"last run {s['last_run']}.)")
    return answer


def in_gazebo(timeout=3.0):
    """True when /sim/ground_truth is published (walle-sim's Gazebo)."""
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    if not rclpy.ok():
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("walle_agent_probe")
    try:
        end = time.time() + timeout
        while time.time() < end:
            if any(n == "/sim/ground_truth" for n, _ in node.get_topic_names_and_types()):
                return True
            time.sleep(0.2)
        return False
    finally:
        node.destroy_node()


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--store", choices=("auto", "robot", "sim"), help="memory store (default "
                   "agent.yaml: auto = sim in Gazebo, else robot)")
    p.add_argument("--store-path", help="read this store file instead (a fixture)")
    p.add_argument("--model", help="Ollama model (default agent.yaml llm.model)")
    p.add_argument("--no-motion", action="store_true", help="memory questions only")
    p.add_argument("--auto-approve", action="store_true",
                   help="approve every motion without asking: Gazebo only (refused otherwise)")
    p.add_argument("--once", help="answer this one prompt and exit")
    p.add_argument("--quiet", action="store_true", help="don't print the agent's steps")
    return p.parse_args(argv)


def main(argv=None):
    a = parse_args(argv)
    cfg = walle_tools.load_config()
    store = a.store or cfg.get("store", "auto")
    gazebo = None
    if store == "auto" or a.auto_approve or not a.no_motion:
        gazebo = in_gazebo()
    if store == "auto":
        store = "sim" if gazebo else "robot"
    if a.auto_approve and not gazebo:
        raise SystemExit("--auto-approve is only allowed in Gazebo (no /sim/ground_truth here).")
    runner = None
    if not a.no_motion:
        from runner import Runner
        runner = Runner(cfg, sim=gazebo)
    walle_tools.configure(store, store_path=a.store_path, cfg=cfg, runner=runner,
                          approve=(lambda d: print(f"[auto-approved in Gazebo] {d}") or True)
                          if a.auto_approve else None)
    agent = build_agent(cfg, model=a.model, verbose=not a.quiet)
    model = a.model or os.environ.get("OLLAMA_MODEL") or cfg["llm"]["model"]
    print(f"WALL-E agent ready: planner {model}, memory '{store}'"
          + (f" ({a.store_path})" if a.store_path else "")
          + (", motion off" if runner is None else
             f", motion {'Gazebo' if gazebo else 'REAL ROBOT'}") + ". Type 'quit' to exit.")
    try:
        if a.once:
            print(f"WALL-E> {respond(agent, a.once)}")
            return 0
        while True:
            try:
                request = input("You> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if request.lower() in ("quit", "exit"):
                break
            if request:
                try:
                    print(f"WALL-E> {respond(agent, request)}")
                except KeyboardInterrupt:
                    if runner is not None:
                        runner.stop()
                    print("\nInterrupted. The robot was stopped.")
    finally:
        if runner is not None:
            runner.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
