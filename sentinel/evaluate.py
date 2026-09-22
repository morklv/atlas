"""Paired policy evaluation. Every result, including regressions, is retained."""
import csv
import json
from dataclasses import asdict
from pathlib import Path
import numpy as np
from .world import World
from .mission import run_mission, MissionConfig
from .render import render_comparison


def evaluate(seeds, conditions, folder, config=None):
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    config = config or MissionConfig()
    rows = []
    for condition in conditions:
        for seed in seeds:
            world = World.generated(seed)
            for policy in ("fixed", "preplanned", "active"):
                result = run_mission(world, policy, condition, config)
                row = {"seed": seed, "condition": condition, "policy": policy,
                       "stop_reason": result.metadata["stop_reason"], **result.metadata["metrics"]}
                rows.append(row)
                print(f"{condition:12} seed={seed:3} {policy:6} coverage={row['coverage']:.1%} distance={row['flight_distance_m']:.1f}m", flush=True)
                # Compact per-run evidence rather than all camera frames.
                (folder / f"{condition}-{seed}-{policy}.json").write_text(
                    json.dumps(result.metadata, indent=2, allow_nan=False))
    with (folder / "results.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    summary = {"config": asdict(config), "seeds": seeds, "conditions": {},
               "scope": "Kinematic depth simulation; no real flight or Gazebo validation"}
    lines = ["# Evaluation results", "",
             "All policies use the same candidate locations, camera, scenes and mission budgets.",
             "The fixed policy follows a predetermined lawnmower order; active selects map-based views.",
             "Preplanned is a stronger baseline: a greedy coverage route computed for empty ground before observations.",
             "A sensor retry consumes time. Yaw changes have zero cost in this simulator.", "",
             "| Condition | Sweep coverage | Preplanned coverage | Active coverage | Gain vs preplanned (pp) | Active wins/ties/losses vs preplanned |",
             "|---|---:|---:|---:|---:|---:|"]
    for condition in conditions:
        fixed = [r for r in rows if r["condition"] == condition and r["policy"] == "fixed"]
        preplanned = [r for r in rows if r["condition"] == condition and r["policy"] == "preplanned"]
        active = [r for r in rows if r["condition"] == condition and r["policy"] == "active"]
        delta = np.array([a["coverage"] - f["coverage"] for a,f in zip(active, preplanned)])
        values = {"mean_fixed_coverage": float(np.mean([r["coverage"] for r in fixed])),
                  "mean_active_coverage": float(np.mean([r["coverage"] for r in active])),
                  "mean_preplanned_coverage": float(np.mean([r["coverage"] for r in preplanned])),
                  "gain_reference": "preplanned",
                  "paired_mean_coverage_gain_pp": float(100*delta.mean()),
                  "paired_std_gain_pp": float(100*delta.std(ddof=1)) if len(delta)>1 else None,
                  "wins": int((delta>1e-9).sum()), "ties": int((np.abs(delta)<=1e-9).sum()),
                  "losses": int((delta< -1e-9).sum()),
                  "mean_fixed_distance_m": float(np.mean([r["flight_distance_m"] for r in fixed])),
                  "mean_preplanned_distance_m": float(np.mean([r["flight_distance_m"] for r in preplanned])),
                  "mean_active_distance_m": float(np.mean([r["flight_distance_m"] for r in active]))}
        summary["conditions"][condition] = values
        lines.append(f"| {condition} | {values['mean_fixed_coverage']:.1%} | {values['mean_preplanned_coverage']:.1%} | {values['mean_active_coverage']:.1%} | {values['paired_mean_coverage_gain_pp']:+.2f} | {values['wins']}/{values['ties']}/{values['losses']} |")
    lines += ["", "## Interpretation limits", "",
              "Coverage counts cells with a sensor endpoint; unknown cells are never labeled free by default.",
              "Accuracy is evaluated only on observed cells. Obstacle recall also counts unknown obstacles as missed.",
              "Inspect all CSV columns: better coverage alone does not establish better classification or shorter flights.",
              "Noise and dropout perturb depth. They do not simulate lighting, dust physics, real optics or localization drift.",
              "These seeds are a reproducible benchmark, not evidence of real-world reliability.",
              "Once used to adjust the algorithm, these layouts are development data; evaluate again on new seeds."]
    (folder / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False))
    (folder / "REPORT.md").write_text("\n".join(lines) + "\n")
    render_comparison(rows, folder)
    return summary
