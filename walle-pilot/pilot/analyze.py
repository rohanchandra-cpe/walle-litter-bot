"""Analyze the logs as the protocol specifies, and write a labeled PILOT summary.

    python3 pilot/analyze.py            # -> data/results/pilot_summary.md, .json, and figures

RQ1/RQ2: paired one-sided t-tests on complete pairs (H1: the difference favors B /
the EKF), two-sided 95 % t intervals, Cohen's dz, an exact paired sign-flip test as
a sensitivity analysis, and Holm correction across RQ1 and RQ2. No normality test
switches the test. RQ3 is descriptive only: availability, valid-reading error,
common-valid paired differences, joint success, latency, and breakdowns.

Accounting: warm-ups and shakedown runs are never analyzed. For each trial the
last attempt counts; an attempt flagged as externally invalid is not analyzed
(its repeat is). Robot failures stay in: their measured endpoints count, and an
unmeasurable endpoint is missing, never filled in.
"""

import itertools
import json
import math
import os

import numpy as np

from common import (LOG_DIR, MOTION_LOG, RANGE_LOG, REPEATABILITY_LOG, RESULTS_DIR, load_config,
                    read_rows, wrap_deg)

DOUBLE_MEASURE_LOG = os.path.join(LOG_DIR, "double_measure.csv")
BANNER = ("PILOT RESULTS: feasibility-scale, not main-study evidence. Do not claim "
          "statistical significance from these numbers; report intervals and what the "
          "full study needs.")


def num(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def last_attempts(rows, key):
    """trial -> its last logged attempt (by attempt number, then file order)."""
    latest = {}
    for row in rows:
        current = latest.get(row[key])
        if current is None or int(row["attempt"]) >= int(current["attempt"]):
            latest[row[key]] = row
    return latest


# ===== statistics =====
def describe(values):
    v = np.array([x for x in values if not math.isnan(x)])
    if len(v) == 0:
        return {"n": 0}
    out = {"n": int(len(v)), "mean": float(v.mean()), "median": float(np.median(v)),
           "sd": float(v.std(ddof=1)) if len(v) > 1 else math.nan, "max": float(v.max())}
    if len(v) > 1:
        from scipy import stats
        half = stats.t.ppf(0.975, len(v) - 1) * out["sd"] / math.sqrt(len(v))
        out["ci95"] = [out["mean"] - half, out["mean"] + half]
    return out


def sign_flip_p(diffs, limit=20, draws=200000, seed=0):
    """One-sided exact (or Monte Carlo for n > limit) paired sign-flip p for mean > 0.
    Assumes each difference is symmetric about 0 under H0 (exchangeable signs)."""
    d = np.array(diffs)
    observed = d.mean()
    if len(d) <= limit:
        flips = np.array(list(itertools.product([1, -1], repeat=len(d))))
    else:
        rng = np.random.default_rng(seed)
        flips = rng.choice([1, -1], size=(draws, len(d)))
    means = (flips * d).mean(axis=1)
    return float(np.mean(means >= observed - 1e-12))


def paired_test(diffs):
    """Paired one-sided t-test of H0: mean <= 0 vs H1: mean > 0, with interval and dz."""
    d = np.array(diffs)
    out = {"n_pairs": int(len(d))}
    if len(d) < 2:
        out["note"] = "fewer than 2 complete pairs: no test"
        return out
    from scipy import stats
    sd = d.std(ddof=1)
    out.update(describe(d))
    out["t"], out["p_one_sided"] = (float(x) for x in stats.ttest_1samp(d, 0.0,
                                                                         alternative="greater"))
    out["dz"] = float(d.mean() / sd) if sd > 0 else math.nan
    out["p_sign_flip"] = sign_flip_p(d)
    return out


def holm(pvalues):
    """Holm-adjusted p-values for a dict name -> p (NaN entries are left out)."""
    items = sorted(((p, k) for k, p in pvalues.items() if not math.isnan(p)))
    adjusted, running = {}, 0.0
    m = len(items)
    for i, (p, k) in enumerate(items):
        running = max(running, min(1.0, (m - i) * p))
        adjusted[k] = running
    return adjusted


# ===== RQ1 and RQ2 =====
def motion_tables(cfg):
    rows = [r for r in read_rows(MOTION_LOG) if r["warmup"] != "1"]
    latest = last_attempts(rows, "trial_id")
    target = 100 * cfg["motion"]["straight_target_m"]
    tol = cfg["tolerances"]
    trials = []
    for trial_id, r in latest.items():
        t = {"trial_id": trial_id, "session": r["session_id"], "task": r["task"],
             "strategy": r["strategy"], "direction": r["direction"], "block": r["block_id"],
             "timestamp": r["timestamp_utc"], "attempts": int(r["attempt"]),
             "external_invalid": r["external_invalid"] == "1",
             "completed": r["completed"] == "1", "gate": r["gate_violation"],
             "measured": r["endpoint_measured"] == "1", "time_s": num(r["completion_s"])}
        if r["task"] == "straight":
            t["error_cm"] = num(r["true_forward_cm"]) - target if t["measured"] else math.nan
            t["lateral_cm"] = num(r["true_lateral_cm"]) if t["measured"] else math.nan
            t["success"] = (t["completed"] and t["measured"] and abs(t["error_cm"]) <= tol["distance_cm"]
                            and abs(t["lateral_cm"]) <= tol["lateral_cm"])
        else:
            true = num(r["true_turn_deg"]) if t["measured"] else math.nan
            t["true_turn_deg"] = true
            t["wheel_error_deg"] = wrap_deg(num(r["wheel_turn_deg"]) - true) if t["measured"] else math.nan
            t["ekf_error_deg"] = wrap_deg(num(r["ekf_turn_deg"]) - true) if t["measured"] else math.nan
        trials.append(t)
    trials.sort(key=lambda t: t["timestamp"])
    return trials


def accounting(trials):
    return {"attempted": len(trials),
            "externally_invalid": sum(t["external_invalid"] for t in trials),
            "failed": sum((not t["completed"]) and not t["external_invalid"] for t in trials),
            "endpoint_missing": sum((not t["measured"]) and not t["external_invalid"] for t in trials),
            "failure_reasons": sorted({t["gate"] for t in trials if t["gate"]})}


def rq1(trials, cfg):
    straight = [t for t in trials if t["task"] == "straight"]
    out = {"by_strategy": {}}
    for s in ("A", "B"):
        sub = [t for t in straight if t["strategy"] == s]
        ok = [t for t in sub if not t["external_invalid"]]
        out["by_strategy"][s] = {
            **accounting(sub),
            "abs_error_cm": describe([abs(t["error_cm"]) for t in ok]),
            "signed_error_cm": describe([t["error_cm"] for t in ok]),
            "abs_lateral_cm": describe([abs(t["lateral_cm"]) for t in ok]),
            "completion_s": describe([t["time_s"] for t in ok]),
            "success": f"{sum(t['success'] for t in ok)}/{len(ok)}"}
    blocks = {}
    for t in straight:
        if not t["external_invalid"] and t["measured"]:
            blocks.setdefault(t["block"], {})[t["strategy"]] = t
    pairs = [b for b in blocks.values() if "A" in b and "B" in b]
    diffs = [abs(b["A"]["error_cm"]) - abs(b["B"]["error_cm"]) for b in pairs]
    out["blocks_planned"] = cfg["study"]["rq1_blocks_per_session"] * cfg["study"]["sessions"]
    out["complete_pairs"] = len(pairs)
    out["test"] = paired_test(diffs)
    out["diffs_cm"] = diffs
    return out


def rq2(trials):
    turns = [t for t in trials if t["task"] == "turn"]
    ok = [t for t in turns if not t["external_invalid"] and t["measured"]]
    diffs = [abs(t["wheel_error_deg"]) - abs(t["ekf_error_deg"]) for t in ok]
    out = {**accounting(turns),
           "wheel_abs_error_deg": describe([abs(t["wheel_error_deg"]) for t in ok]),
           "ekf_abs_error_deg": describe([abs(t["ekf_error_deg"]) for t in ok]),
           "wheel_signed_error_deg": describe([t["wheel_error_deg"] for t in ok]),
           "ekf_signed_error_deg": describe([t["ekf_error_deg"] for t in ok]),
           "true_turn_magnitude_deg": describe([abs(t["true_turn_deg"]) for t in ok]),
           "by_direction": {}, "test": paired_test(diffs), "diffs_deg": diffs}
    for d in ("ccw", "cw"):
        sub = [t for t in ok if t["direction"] == d]
        out["by_direction"][d] = {
            "n": len(sub), "wheel_abs_error_deg": describe([abs(t["wheel_error_deg"]) for t in sub]),
            "ekf_abs_error_deg": describe([abs(t["ekf_error_deg"]) for t in sub])}
    return out


# ===== RQ3 =====
def rq3(cfg):
    latest = last_attempts(read_rows(RANGE_LOG), "placement_id")
    rows = [r for r in latest.values() if r["external_invalid"] != "1"]
    tol = cfg["tolerances"]["ape_pct"]
    out = {"placements_planned": cfg["study"]["rq3_placements"], "placements_analyzed": len(rows),
           "externally_invalid": sum(r["external_invalid"] == "1" for r in latest.values()),
           "methods": {}, "breakdown": {}}
    records = []
    for r in rows:
        truth = num(r["true_range_m"])
        rec = {"object": r["object"], "angle": r["angle_deg"], "distance": r["distance_m"],
               "truth": truth}
        for key in ("us", "cam"):
            valid = r[f"{key}_valid"] == "1"
            est = num(r[f"{key}_range_m"]) if valid else math.nan
            rec[f"{key}_valid"] = valid
            rec[f"{key}_ae"] = abs(est - truth) if valid else math.nan
            rec[f"{key}_ape"] = 100 * abs(est - truth) / truth if valid else math.nan
            rec[f"{key}_ms"] = num(r[f"{key}_latency_ms"])
        records.append(rec)
    for key, name in (("us", "ultrasonic"), ("cam", "camera")):
        valid = [x for x in records if x[f"{key}_valid"]]
        out["methods"][name] = {
            "valid": f"{len(valid)}/{len(records)}",
            "ae_m": describe([x[f"{key}_ae"] for x in valid]),
            "ape_pct": describe([x[f"{key}_ape"] for x in valid]),
            "joint_success": f"{sum(x[f'{key}_ape'] <= tol for x in valid)}/{len(records)}",
            "latency_ms": describe([x[f"{key}_ms"] for x in records])}
    common = [x for x in records if x["us_valid"] and x["cam_valid"]]
    out["common_valid"] = {
        "n": len(common),
        "ae_diff_m_camera_minus_ultrasonic": describe([x["cam_ae"] - x["us_ae"] for x in common]),
        "ape_diff_pp_camera_minus_ultrasonic": describe([x["cam_ape"] - x["us_ape"] for x in common])}
    for factor in ("object", "angle", "distance"):
        table = {}
        for level in sorted({x[factor] for x in records}):
            sub = [x for x in records if x[factor] == level]
            table[level] = {name: {"valid": f"{sum(x[f'{k}_valid'] for x in sub)}/{len(sub)}",
                                   "median_ape_pct": describe([x[f"{k}_ape"] for x in sub
                                                               if x[f"{k}_valid"]]).get("median", math.nan)}
                            for k, name in (("us", "ultrasonic"), ("cam", "camera"))}
        out["breakdown"][factor] = table
    return out, records


# ===== ground-truth quality =====
def ground_truth_quality():
    rows = read_rows(REPEATABILITY_LOG)
    out = {}
    groups = {}
    for r in rows:
        groups.setdefault((r["quantity"], r["measurer"], r["pose_id"]), []).append(num(r["value"]))
    for (quantity, measurer, _), values in groups.items():
        if len(values) > 1:
            out.setdefault(quantity, {}).setdefault(measurer, []).append(float(np.std(values, ddof=1)))
    repeatability = {q: {m: float(np.sqrt(np.mean(np.square(sds)))) for m, sds in by.items()}
                     for q, by in out.items()}
    second = read_rows(DOUBLE_MEASURE_LOG)
    main = last_attempts([r for r in read_rows(MOTION_LOG)], "trial_id")
    disagreements = {}
    for r in second:
        first = main.get(r["trial_id"], {}).get(r["quantity"])
        if first not in (None, ""):
            disagreements.setdefault(r["quantity"], []).append(abs(num(r["value"]) - num(first)))
    return {"pooled_repeat_sd": repeatability,
            "double_measured_mean_abs_disagreement": {
                q: {"n": len(v), "mean": float(np.mean(v))} for q, v in disagreements.items()}}


# ===== figures =====
def figures(trials, r1, records, cfg):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    os.makedirs(RESULTS_DIR, exist_ok=True)
    tol = cfg["tolerances"]
    paths = []
    straight = [t for t in trials if t["task"] == "straight" and not t["external_invalid"]
                and t["measured"]]
    if straight:
        fig, ax = plt.subplots(1, 2, figsize=(9, 3.6))
        blocks = {}
        for t in straight:
            blocks.setdefault(t["block"], {})[t["strategy"]] = abs(t["error_cm"])
        for b in blocks.values():
            if len(b) == 2:
                ax[0].plot([0, 1], [b["A"], b["B"]], color="0.7", lw=1, zorder=1)
        for i, s in enumerate("AB"):
            ys = [abs(t["error_cm"]) for t in straight if t["strategy"] == s]
            ax[0].scatter([i] * len(ys), ys, zorder=2, label=s)
        ax[0].axhline(tol["distance_cm"], ls="--", color="k", lw=1)
        ax[0].set(xticks=[0, 1], xticklabels=["A timed", "B encoder"], xlim=(-0.5, 1.5),
                  ylabel="|endpoint error| (cm)", title="RQ1 paired blocks (PILOT)")
        for s in "AB":
            pts = [(i, t["error_cm"]) for i, t in enumerate(straight) if t["strategy"] == s]
            ax[1].scatter([p[0] for p in pts], [p[1] for p in pts], label=s)
        ax[1].axhline(0, color="k", lw=0.8)
        ax[1].set(xlabel="run order", ylabel="signed error (cm)", title="Signed error by run order")
        ax[1].legend()
        fig.tight_layout()
        paths.append(os.path.join(RESULTS_DIR, "rq1_endpoint_error.png"))
        fig.savefig(paths[-1], dpi=150)
        plt.close(fig)

    turns = [t for t in trials if t["task"] == "turn" and not t["external_invalid"] and t["measured"]]
    if turns:
        fig, ax = plt.subplots(figsize=(4.8, 3.6))
        for t in turns:
            ax.plot([0, 1], [abs(t["wheel_error_deg"]), abs(t["ekf_error_deg"])],
                    color="tab:blue" if t["direction"] == "ccw" else "tab:orange", marker="o")
        ax.axhline(tol["heading_deg"], ls="--", color="k", lw=1)
        ax.set(xticks=[0, 1], xticklabels=["wheel only", "EKF"], xlim=(-0.3, 1.3),
               ylabel="|heading error| (deg)", title="RQ2 same-turn pairs (PILOT)\nblue CCW, orange CW")
        fig.tight_layout()
        paths.append(os.path.join(RESULTS_DIR, "rq2_heading_error.png"))
        fig.savefig(paths[-1], dpi=150)
        plt.close(fig)

    if records:
        angles = sorted({x["angle"] for x in records}, key=float)
        fig, axes = plt.subplots(1, len(angles), figsize=(4.6 * len(angles), 3.6), squeeze=False)
        for ax, angle in zip(axes[0], angles):
            sub = [x for x in records if x["angle"] == angle]
            top = max([x[f"{k}_ape"] for x in records for k in ("us", "cam")
                       if not math.isnan(x[f"{k}_ape"])] + [tol["ape_pct"] * 2])
            for k, name, offset in (("us", "ultrasonic", -0.02), ("cam", "camera", 0.02)):
                valid = [x for x in sub if x[f"{k}_valid"]]
                bad = [x for x in sub if not x[f"{k}_valid"]]
                line = ax.scatter([x["truth"] + offset for x in valid],
                                  [x[f"{k}_ape"] for x in valid], label=name)
                ax.scatter([x["truth"] + offset for x in bad], [top * 1.08] * len(bad),
                           marker="x", color=line.get_facecolor()[0])
            ax.axhline(tol["ape_pct"], ls="--", color="k", lw=1)
            ax.set(xlabel="true range (m)", ylabel="APE (%)",
                   title=f"RQ3 at {angle} deg (x = invalid reading) PILOT")
            ax.legend()
        fig.tight_layout()
        paths.append(os.path.join(RESULTS_DIR, "rq3_ranging.png"))
        fig.savefig(paths[-1], dpi=150)
        plt.close(fig)
    return paths


# ===== report =====
def fmt_desc(d, unit=""):
    if not d or d.get("n", 0) == 0:
        return "no data"
    text = f"n={d['n']}, mean {d['mean']:.2f}{unit}, median {d['median']:.2f}{unit}"
    if not math.isnan(d.get("sd", math.nan)):
        text += f", SD {d['sd']:.2f}"
    if "ci95" in d:
        text += f", 95% CI [{d['ci95'][0]:.2f}, {d['ci95'][1]:.2f}]"
    return text + f", max {d['max']:.2f}{unit}"


def fmt_test(t, unit, sesoi):
    if t.get("n_pairs", 0) < 2:
        return f"{t.get('n_pairs', 0)} complete pairs: no test."
    lines = [f"- complete pairs: {t['n_pairs']}",
             f"- mean paired difference {t['mean']:+.2f} {unit} (positive favors the second "
             f"option), unadjusted two-sided 95% t interval [{t['ci95'][0]:+.2f}, {t['ci95'][1]:+.2f}]",
             f"- one-sided paired t: t = {t['t']:.2f}, p = {t['p_one_sided']:.4f}; Holm-adjusted p = "
             f"{t.get('p_holm', math.nan):.4f}; Cohen's dz = {t['dz']:.2f}",
             f"- sensitivity: paired sign-flip p = {t['p_sign_flip']:.4f} (assumes symmetric, "
             "exchangeable signs; limited by n)",
             f"- effect of interest: {sesoi} {unit}. Interpret the interval against it; a "
             "nonsignificant result does not establish equivalence."]
    return "\n".join(lines)


def write_report(cfg, r1, r2, r3, gt, figs):
    eoi = cfg["effects_of_interest"]
    lines = [f"# WALL-E pilot summary\n\n**{BANNER}**\n"]
    lines.append("## RQ1: endpoint distance error (A timed vs B encoder-targeted)\n")
    for s, d in r1["by_strategy"].items():
        lines.append(f"**Strategy {s}**: attempted {d['attempted']}, externally invalid "
                     f"{d['externally_invalid']}, failed {d['failed']} "
                     f"({', '.join(d['failure_reasons']) or 'none'}), endpoint missing "
                     f"{d['endpoint_missing']}, success {d['success']}  ")
        lines.append(f"|error|: {fmt_desc(d['abs_error_cm'], ' cm')}  ")
        lines.append(f"signed error: {fmt_desc(d['signed_error_cm'], ' cm')}  ")
        lines.append(f"|lateral|: {fmt_desc(d['abs_lateral_cm'], ' cm')}  ")
        lines.append(f"completion time: {fmt_desc(d['completion_s'], ' s')}\n")
    lines.append(f"Paired blocks planned {r1['blocks_planned']}, complete pairs {r1['complete_pairs']} "
                 "(failures with measurable endpoints included).\n")
    lines.append(fmt_test(r1["test"], "cm", eoi["distance_cm"]) + "\n")
    lines.append("## RQ2: heading-estimate error (wheel only vs EKF, same turn)\n")
    lines.append(f"attempted {r2['attempted']}, externally invalid {r2['externally_invalid']}, "
                 f"failed {r2['failed']}, endpoint missing {r2['endpoint_missing']}  ")
    lines.append(f"true turn magnitude: {fmt_desc(r2['true_turn_magnitude_deg'], ' deg')}  ")
    lines.append(f"wheel |error|: {fmt_desc(r2['wheel_abs_error_deg'], ' deg')}  ")
    lines.append(f"EKF |error|: {fmt_desc(r2['ekf_abs_error_deg'], ' deg')}  ")
    lines.append(f"wheel signed: {fmt_desc(r2['wheel_signed_error_deg'], ' deg')}  ")
    lines.append(f"EKF signed: {fmt_desc(r2['ekf_signed_error_deg'], ' deg')}\n")
    for d, v in r2["by_direction"].items():
        lines.append(f"- {d}: n={v['n']}; wheel {fmt_desc(v['wheel_abs_error_deg'], ' deg')}; "
                     f"EKF {fmt_desc(v['ekf_abs_error_deg'], ' deg')}")
    lines.append("\n" + fmt_test(r2["test"], "deg", eoi["heading_deg"]) + "\n")
    lines.append("## RQ3: ranging (exploratory, descriptive only)\n")
    lines.append(f"placements planned {r3['placements_planned']}, analyzed {r3['placements_analyzed']}, "
                 f"externally invalid {r3['externally_invalid']}\n")
    for name, m in r3["methods"].items():
        lines.append(f"**{name}**: valid {m['valid']}, joint success (valid and APE <= "
                     f"{cfg['tolerances']['ape_pct']}%) {m['joint_success']}  ")
        lines.append(f"AE: {fmt_desc(m['ae_m'], ' m')}  ")
        lines.append(f"APE: {fmt_desc(m['ape_pct'], ' %')}  ")
        lines.append(f"latency: {fmt_desc(m['latency_ms'], ' ms')}\n")
    c = r3["common_valid"]
    lines.append(f"Common-valid placements: {c['n']} (may select easier placements; read with "
                 "availability).  ")
    lines.append(f"AE difference, camera - ultrasonic: {fmt_desc(c['ae_diff_m_camera_minus_ultrasonic'], ' m')}  ")
    lines.append(f"APE difference, camera - ultrasonic: {fmt_desc(c['ape_diff_pp_camera_minus_ultrasonic'], ' pp')}"
                 f" (reference effect of interest {eoi['ape_pp']} pp)\n")
    for factor, table in r3["breakdown"].items():
        lines.append(f"\n| {factor} | ultrasonic valid | ultrasonic median APE | camera valid | camera median APE |")
        lines.append("|---|---|---|---|---|")
        for level, v in table.items():
            lines.append(f"| {level} | {v['ultrasonic']['valid']} | {v['ultrasonic']['median_ape_pct']:.1f} "
                         f"| {v['camera']['valid']} | {v['camera']['median_ape_pct']:.1f} |")
    lines.append("\n## Ground-truth quality\n")
    lines.append(f"repeatability (pooled SD by quantity and measurer): {json.dumps(gt['pooled_repeat_sd'])}  ")
    lines.append(f"double-measured disagreement: {json.dumps(gt['double_measured_mean_abs_disagreement'])}\n")
    if figs:
        lines.append("## Figures\n")
        lines.extend(f"![{os.path.basename(p)}]({os.path.basename(p)})" for p in figs)
    path = os.path.join(RESULTS_DIR, "pilot_summary.md")
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")
    return path


def main():
    cfg = load_config()
    trials = motion_tables(cfg)
    r1, r2 = rq1(trials, cfg), rq2(trials)
    adjusted = holm({"rq1": r1["test"].get("p_one_sided", math.nan),
                     "rq2": r2["test"].get("p_one_sided", math.nan)})
    r1["test"]["p_holm"] = adjusted.get("rq1", math.nan)
    r2["test"]["p_holm"] = adjusted.get("rq2", math.nan)
    r3_out, records = rq3(cfg)
    gt = ground_truth_quality()
    os.makedirs(RESULTS_DIR, exist_ok=True)
    figs = figures(trials, r1, records, cfg)
    report = write_report(cfg, r1, r2, r3_out, gt, figs)
    with open(os.path.join(RESULTS_DIR, "pilot_summary.json"), "w") as f:
        json.dump({"label": BANNER, "rq1": r1, "rq2": r2, "rq3": r3_out,
                   "ground_truth": gt}, f, indent=2, default=str)
    print(BANNER)
    print(f"Wrote {report}, pilot_summary.json, and {len(figs)} figure(s) in {RESULTS_DIR}")


if __name__ == "__main__":
    main()
