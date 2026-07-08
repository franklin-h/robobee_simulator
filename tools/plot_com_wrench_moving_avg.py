#!/usr/bin/env python3
"""Plot the running (expanding) average of the RoboBee COM wrench.

Variant of plot_com_wrench.py. Instead of plotting the raw signals in stacked
subplots, this plots the expanding moving average of each channel: at every
timestep i (with i >= 50) the plotted value is the mean of samples 50..i. Each
channel (z-axis thrust, roll/pitch/yaw torque) is drawn in its own separate
figure rather than as a subplot.

Usage:
  python3 tools/plot_com_wrench_moving_avg.py                # one-shot
  python3 tools/plot_com_wrench_moving_avg.py --live         # refresh continuously
  python3 tools/plot_com_wrench_moving_avg.py --start 50     # first sample index
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
]

# Manual y-axis limits per channel. Set (low, high) to override auto-scaling;
# leave as None to auto-scale that channel from its plotted running average.
YLIMS = {
    "thrust_z_N": None,
    "roll_torque_Nm": None,
    "pitch_torque_Nm": (-5e-7,5e-7),
    "yaw_torque_Nm": None,
}


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


def expanding_average(x_values, y_values, start):
    """Return (x, running_avg) where running_avg[i] = mean(y[start..i]).

    Only samples from index `start` onward are emitted; each plotted point is
    the average of every sample from `start` up to and including that point.
    """
    x_out = []
    avg_out = []
    running_sum = 0.0
    count = 0
    for i, (t, y) in enumerate(zip(x_values, y_values)):
        if i < start:
            continue
        running_sum += y
        count += 1
        x_out.append(t)
        avg_out.append(running_sum / count)
    return x_out, avg_out


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


def compute_averages(rows, start):
    averages = {}
    for key, _label, _unit, _color in CHANNELS:
        _x, y = xy_series(rows, key)
        averages[key] = average(y[start:])
    return averages


def print_averages(averages, sample_count, span, start):
    print("=" * 52)
    print(f"Averages over samples {start}..{sample_count} "
          f"({span:.4f} s of sim time):")
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


def draw(figures, rows, averages, start):
    for (fig, axis), (key, label, unit, color) in zip(figures, CHANNELS):
        axis.clear()
        x, y = xy_series(rows, key)
        x_avg, y_avg = expanding_average(x, y, start)
        if y_avg:
            axis.plot(x_avg, y_avg, color=color, linewidth=1.2)
            limits = YLIMS.get(key) or padded_limits(y_avg)
            if limits is not None:
                axis.set_ylim(*limits)
            # Pin the x-axis to the first plotted (post-`start`) sample so the
            # transient is visibly dropped instead of just being covered by
            # autoscale padding.
            if x_avg[0] < x_avg[-1]:
                axis.set_xlim(x_avg[0], x_avg[-1])
        mean = averages.get(key)
        if mean is not None:
            axis.axhline(mean, color="k", linestyle="--", linewidth=0.9,
                         label=f"final avg = {mean:+.4e} {unit}")
            axis.legend(loc="upper right", fontsize="small")
        axis.set_ylabel(f"running-avg {label} [{unit}]")
        axis.set_xlabel("simulation time [s]")
        axis.grid(True, alpha=0.3)
        axis.set_title(f"Running average of {label} about the RoboBee COM "
                       f"(from sample {start})")
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
        "--start",
        type=int,
        default=200,
        help="First sample index included in the running average.",
    )
    parser.add_argument(
        "--last",
        type=float,
        default=None,
        help="Average/plot only the last N seconds of sim time.",
    )
    parser.add_argument(
        "--live",
        action="store_true",
        help="Refresh the plots and averages continuously.",
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
        help="Only print averages; do not open plot windows.",
    )
    args = parser.parse_args()

    path = pathlib.Path(args.csv_path)

    if args.no_plot:
        if not path.exists():
            raise SystemExit(f"{path} does not exist yet.")
        rows = window_rows(read_rows(path), args.last)
        if not rows:
            raise SystemExit(f"No usable rows in {path}.")
        print_averages(compute_averages(rows, args.start), len(rows),
                       time_span(rows), args.start)
        return

    try:
        import matplotlib.pyplot as plt
        
    except ImportError as exc:
        raise SystemExit(
            "matplotlib is required: python3 -m pip install matplotlib"
        ) from exc

    # One separate figure per channel instead of stacked subplots.
    figures = []
    for _key, label, _unit, _color in CHANNELS:
        fig, axis = plt.subplots(1, 1, figsize=(9, 4), num=label)
        figures.append((fig, axis))

    if not args.live:
        if not path.exists():
            raise SystemExit(f"{path} does not exist yet.")
        rows = window_rows(read_rows(path), args.last)
        if not rows:
            raise SystemExit(f"No usable rows in {path}.")
        averages = compute_averages(rows, args.start)
        print_averages(averages, len(rows), time_span(rows), args.start)
        draw(figures, rows, averages, args.start)
        
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
        averages = compute_averages(rows, args.start)
        print_averages(averages, len(rows), time_span(rows), args.start)
        draw(figures, rows, averages, args.start)
        plt.pause(0.001)
        time.sleep(args.period)


if __name__ == "__main__":
    main()
