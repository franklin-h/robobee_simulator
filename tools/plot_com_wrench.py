#!/usr/bin/env python3
"""Plot the net aerodynamic wrench about the RoboBee COM and report averages.

Reads the CSV written by visualize_robobee_aeromechanical (the fixed / welded
build) and plots the net z-axis thrust plus the roll, pitch, and yaw torques
about the center of mass. The time-average of each signal is printed to stdout
and annotated on the plot.

Usage:
  python3 tools/plot_com_wrench.py                # one-shot: plot + averages
  python3 tools/plot_com_wrench.py --live         # refresh continuously
  python3 tools/plot_com_wrench.py --last 0.1     # average only the last 0.1 s
"""

import argparse
import csv
import math
import pathlib
import sys
import time


CHANNELS = [
    ("thrust_z_N", "z-axis thrust", "N", "tab:blue"),
    ("roll_torque_Nm", "roll torque", "N·m", "tab:red"),
    ("pitch_torque_Nm", "pitch torque", "N·m", "tab:green"),
    ("yaw_torque_Nm", "yaw torque", "N·m", "tab:purple"),
    # Controller axes: +x forward, +y left. Absent in CSVs recorded before
    # force_x_N/force_y_N were added; these panels then show "(no data)".
    ("force_x_N", "x-axis (fwd) force", "N", "tab:orange"),
    ("force_y_N", "y-axis (left) force", "N", "tab:brown"),
]


def set_csv_field_limit():
    limit = sys.maxsize
    while True:
        try:
            csv.field_size_limit(limit)
            return
        except OverflowError:
            limit //= 10


def read_rows(path):
    set_csv_field_limit()
    rows = []
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        while True:
            try:
                row = next(reader)
            except StopIteration:
                break
            except csv.Error:
                break
            if row and all(value is not None for value in row.values()):
                rows.append(row)
    return rows


def parse_float(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def xy_series(rows, y_key):
    x_values = []
    y_values = []
    for row in rows:
        t = parse_float(row.get("time_s"))
        y = parse_float(row.get(y_key))
        if t is None or y is None:
            continue
        x_values.append(t)
        y_values.append(y)
    return x_values, y_values


def average(values):
    finite = [v for v in values if v is not None and math.isfinite(v)]
    if not finite:
        return None
    return sum(finite) / len(finite)


def padded_limits(values):
    finite = [v for v in values if math.isfinite(v)]
    if not finite:
        return None
    low = min(finite)
    high = max(finite)
    if low == high:
        padding = max(abs(low) * 0.05, 1.0e-9)
    else:
        padding = 0.05 * (high - low)
    return low - padding, high + padding


def window_rows(rows, last_seconds):
    if last_seconds is None:
        return rows
    times = [parse_float(row.get("time_s")) for row in rows]
    times = [t for t in times if t is not None]
    if not times:
        return rows
    t_min = times[-1] - last_seconds
    return [
        row for row in rows
        if (parse_float(row.get("time_s")) is not None and
            parse_float(row.get("time_s")) >= t_min)
    ]


def compute_averages(rows):
    averages = {}
    for key, _label, _unit, _color in CHANNELS:
        _x, y = xy_series(rows, key)
        averages[key] = average(y[50:])
    return averages


def print_averages(averages, sample_count, span):
    print("=" * 52)
    print(f"Averages over {sample_count} samples ({span:.4f} s of sim time):")
    for key, label, unit, _color in CHANNELS:
        value = averages.get(key)
        if value is None:
            print(f"  {label:<14s}: (no data)")
        else:
            print(f"  {label:<14s}: {value:+.6e} {unit}")
    print("=" * 52)


def time_span(rows):
    times = [parse_float(row.get("time_s")) for row in rows]
    times = [t for t in times if t is not None]
    if len(times) < 2:
        return 0.0
    return times[-1] - times[0]


def draw(fig, axes, rows, averages):
    for axis, (key, label, unit, color) in zip(axes, CHANNELS):
        axis.clear()
        x, y = xy_series(rows, key)
        if y:
            axis.plot(x[60:], y[60:], color=color, linewidth=1.0)
            limits = padded_limits(y)
            if limits is not None:
                axis.set_ylim(*limits)
        mean = averages.get(key)
        if mean is not None:
            axis.axhline(mean, color="k", linestyle="--", linewidth=0.9,
                         label=f"avg = {mean:+.4e} {unit}")
            axis.legend(loc="upper right", fontsize="small")
        axis.set_ylabel(f"{label} [{unit}]")
        axis.grid(True, alpha=0.3)
    axes[-1].set_xlabel("simulation time [s]")
    fig.suptitle("Net aerodynamic wrench about the RoboBee COM")
    fig.tight_layout()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "csv_path",
        nargs="?",
        default="/tmp/robobee_com_wrench.csv",
        help="CSV written by visualize_robobee_aeromechanical.",
    )
    parser.add_argument(
        "--last",
        type=float,
        default=0.02,
        help="Average/plot only the last N seconds of sim time.",
    )
    parser.add_argument(
        "--live",
        action="store_true",
        help="Refresh the plot and averages continuously.",
    )
    parser.add_argument(
        "--period",
        type=float,
        default=0.5,
        help="Refresh period in real seconds when --live is set.",
    )
    parser.add_argument(
        "--no-plot",
        action="store_true",
        help="Only print averages; do not open a plot window.",
    )
    args = parser.parse_args()

    path = pathlib.Path(args.csv_path)

    if args.no_plot:
        if not path.exists():
            raise SystemExit(f"{path} does not exist yet.")
        rows = window_rows(read_rows(path), args.last)
        if not rows:
            raise SystemExit(f"No usable rows in {path}.")
        print_averages(compute_averages(rows), len(rows), time_span(rows))
        return

    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit(
            "matplotlib is required: python3 -m pip install matplotlib"
        ) from exc

    fig, axes = plt.subplots(len(CHANNELS), 1, sharex=True, figsize=(10, 12))

    if not args.live:
        if not path.exists():
            raise SystemExit(f"{path} does not exist yet.")
        rows = window_rows(read_rows(path), args.last)
        if not rows:
            raise SystemExit(f"No usable rows in {path}.")
        averages = compute_averages(rows)
        print_averages(averages, len(rows), time_span(rows))
        draw(fig, axes, rows, averages)
        plt.show()
        return

    plt.ion()
    while True:
        if not path.exists():
            print(f"Waiting for {path}...")
            time.sleep(args.period)
            continue
        rows = window_rows(read_rows(path), args.last)
        if not rows:
            time.sleep(args.period)
            continue
        averages = compute_averages(rows)
        print_averages(averages, len(rows), time_span(rows))
        draw(fig, axes, rows, averages)
        plt.pause(0.001)
        time.sleep(args.period)


if __name__ == "__main__":
    main()
