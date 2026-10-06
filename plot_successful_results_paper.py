#!/usr/bin/env python3
"""Plot results produced by the float8 discrete-validation runner.

This viewer intentionally does not look for DynoBench/FCL validation fields.
It plots the sampled-success trajectory and the exact sphere geometry exported
by the run.
"""
import argparse
import csv
import html
import math
import re
from pathlib import Path


CASES = (
    "uni_bugtrap",
    "uni_wall",
    "quad2d_bugtrap",
    "quad2d_hole",
    "quad3d_block",
    "quad3d_window",
)


def parse_vector(text):
    return [float(x.strip()) for x in text.split(",") if x.strip()]


def read_simple_yaml(path):
    values = {"states": [], "actions": [], "times": []}
    section = None
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if line in ("states:", "actions:"):
            section = line[:-1]
            continue
        if line.startswith("- "):
            if section in ("states", "actions"):
                values[section].append(parse_vector(line[2:].strip("[]")))
            continue
        if line.startswith("times:"):
            values["times"] = parse_vector(line.split(":", 1)[1].strip().strip("[]"))
    if not values["states"] or not values["times"]:
        raise ValueError(f"trajectory has no states/times: {path}")
    return values


def read_geometry(path):
    spheres = []
    pattern = re.compile(
        r"center:\s*\[([^]]+)\],\s*radius:\s*([-+0-9.eE]+)"
    )
    for raw in path.read_text(encoding="utf-8").splitlines():
        match = pattern.search(raw)
        if match:
            center = parse_vector(match.group(1))
            while len(center) < 3:
                center.append(0.0)
            spheres.append((center, float(match.group(2))))
    if not spheres:
        raise ValueError(f"geometry has no exported spheres: {path}")
    return spheres


def read_result(case_dir, run):
    result_file = case_dir / "results.csv"
    with result_file.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    row = next((item for item in rows if int(item["run"]) == run), None)
    if row is None:
        raise ValueError(f"run {run} is absent from {result_file}")
    if row.get("success") != "1" or row.get("validation_backend") != "simd_discrete_float8":
        raise ValueError(
            f"{case_dir.name} run {run} is not a sampled SIMD success "
            f"(success={row.get('success')}, backend={row.get('validation_backend')})"
        )
    return row


def draw_case(case_dir, row, run, output_dir):
    import matplotlib.pyplot as plt

    trajectory = read_simple_yaml(case_dir / f"trajectory_{run}.yaml")
    spheres = read_geometry(case_dir / "geometry.yaml")
    state_dim = len(trajectory["states"][0])
    positions = [state[:3] for state in trajectory["states"]]
    is_3d = state_dim == 13
    figure = plt.figure(figsize=(9, 7))
    if is_3d:
        axis = figure.add_subplot(111, projection="3d")
        axis.plot(
            [point[0] for point in positions],
            [point[1] for point in positions],
            [point[2] for point in positions],
            color="#1167c4",
            linewidth=2,
            label="sampled SIMD path",
        )
        # A sphere union is the authoritative geometry. The center cloud is
        # deliberately used instead of replacing it with the old FCL boxes.
        stride = max(1, len(spheres) // 8000)
        obstacle_points = [sphere[0] for sphere in spheres[::stride]]
        axis.scatter(
            [point[0] for point in obstacle_points],
            [point[1] for point in obstacle_points],
            [point[2] for point in obstacle_points],
            s=3,
            alpha=0.18,
            color="#6f7782",
            label="sphere-union geometry",
        )
        axis.set_zlabel("z (m)")
    else:
        axis = figure.add_subplot(111)
        axis.plot(
            [point[0] for point in positions],
            [point[1] for point in positions],
            color="#1167c4",
            linewidth=2,
            label="sampled SIMD path",
        )
        for center, radius in spheres:
            axis.add_patch(
                plt.Circle(
                    (center[0], center[1]), radius, color="#6f7782", alpha=0.18
                )
            )
        axis.set_ylabel("y (m)" if state_dim == 3 else "z (m)")
        axis.set_aspect("equal", adjustable="datalim")
    start, goal = positions[0], positions[-1]
    if is_3d:
        axis.scatter(*start, color="#168a45", s=45, label="start")
        axis.scatter(*goal, color="#c83333", marker="*", s=90, label="goal")
        axis.set_xlabel("x (m)")
        axis.set_ylabel("y (m)")
    else:
        axis.scatter(start[0], start[1], color="#168a45", s=45, label="start")
        axis.scatter(goal[0], goal[1], color="#c83333", marker="*", s=90, label="goal")
        axis.set_xlabel("x (m)")
    backend = row["validation_backend"]
    axis.set_title(
        f"{case_dir.name} | run {run} | {backend}\n"
        f"first solution: {float(row['first_solution_time']):.6f} s | "
        f"path length: {float(row['path_length']):.6f} m"
    )
    axis.grid(True, alpha=0.25)
    axis.legend(loc="best")
    figure.tight_layout()
    image = output_dir / f"{case_dir.name}.png"
    figure.savefig(image, dpi=160)
    plt.close(figure)
    return image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--experiment", type=Path, required=True)
    parser.add_argument("--cases", nargs="+", choices=CASES, default=list(CASES))
    parser.add_argument("--run", type=int, default=0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    rows = []
    images = []
    for case in dict.fromkeys(args.cases):
        case_dir = args.experiment / case
        row = read_result(case_dir, args.run)
        image = draw_case(case_dir, row, args.run, args.output)
        images.append(image.name)
        rows.append(
            {
                "environment": case,
                "run": row["run"],
                "seed": row["seed"],
                "sampled_success": row["success"],
                "validation_backend": row["validation_backend"],
                "first_solution_time_s": row["first_solution_time"],
                "planning_time_s": row["planning_time"],
                "path_length_m": row["path_length"],
                "trajectory_duration_s": row["trajectory_duration"],
                "cost": row["cost"],
                "simd_validation_queries": row.get("simd_validation_queries", ""),
                "simd_validation_batches": row.get("simd_validation_batches", ""),
            }
        )
    metrics = args.output / "metrics.csv"
    with metrics.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    links = "\n".join(
        f'<p><a href="{html.escape(image)}"><img src="{html.escape(image)}" width="800"></a></p>'
        for image in images
    )
    (args.output / "index.html").write_text(
        "<!doctype html><meta charset='utf-8'><title>SIMD sampled results</title>"
        "<h1>SIMD discrete float8 sampled results</h1>" + links,
        encoding="utf-8",
    )
    print(f"Output: {args.output}")
    for row in rows:
        print(
            f"{row['environment']}: first_solution={row['first_solution_time_s']} s | "
            f"path_length={row['path_length_m']} m | duration={row['trajectory_duration_s']} s"
        )


if __name__ == "__main__":
    main()
