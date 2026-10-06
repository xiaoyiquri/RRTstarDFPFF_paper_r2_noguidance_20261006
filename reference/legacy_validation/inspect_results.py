#!/usr/bin/env python3
"""Read an existing six-environment run; never rerun a planner."""
import argparse
import csv
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    args = parser.parse_args()
    groups = [
        ["run", "success", "nodes", "iterations", "planning_time", "failure_reason"],
        ["closest_goal_distance", "goal_queries", "goal_valid"],
        ["pff_queries", "pff_evaluations", "flat_bound_rejected", "df_rejected"],
        ["dynobench_validations", "state_bound_rejected", "control_bound_rejected",
         "collision_rejected", "dynamics_rejected"],
        ["pff_time", "df_time", "dynobench_time", "simd_active"],
        ["candidate_prechecks", "candidate_precheck_rejected", "collision_precheck_queries",
         "collision_precheck_rejected", "goal_cache_hits", "goal_cache_exhausted", "dense_reconstructions"],
    ]
    cases = ("uni_bugtrap", "uni_wall", "quad2d_bugtrap", "quad2d_hole", "quad3d_block", "quad3d_window")
    manifest = args.directory / "manifest.json"
    if manifest.is_file():
        selected = json.loads(manifest.read_text(encoding="utf-8")).get("cases", cases)
        cases = tuple(name for name in cases if name in selected)
    for name in cases:
        path = args.directory / name / "results.csv"
        if not path.is_file():
            print(name + ": no results.csv")
            continue
        print("\n===== " + name + " =====")
        with path.open(newline="", encoding="utf-8") as stream:
            for row in csv.DictReader(stream):
                for fields in groups:
                    print(" | ".join(k + "=" + row.get(k, "missing") for k in fields))
                reasons = sorted(((k, int(v)) for k, v in row.items()
                                  if k != "df_rejected" and k.startswith("df_") and k.endswith("_rejected")),
                                 key=lambda item: item[1], reverse=True)
                print("DF reasons:", ", ".join(k + "=" + str(v) for k, v in reasons if v))
        validation = args.directory / name / "validation.csv"
        if validation.is_file():
            with validation.open(newline="", encoding="utf-8") as stream:
                for row in csv.DictReader(stream):
                    print("validation:", dict(row))


if __name__ == "__main__":
    main()
