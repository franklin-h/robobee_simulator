#!/usr/bin/env python3
"""Live plot slider actual and desired positions from the simulator."""

import argparse
import csv
import math
import pathlib
import sys
import time


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


def row_value(row, key):
    return parse_float(row.get(key))


def row_value_any(row, keys):
    for key in keys:
        value = row_value(row, key)
        if value is not None:
            return value
    return None


def xy_series(rows, y_keys, scale=1.0):
    x_values = []
    y_values = []
    for row in rows:
        t = row_value(row, "time_s")
        y = row_value_any(row, y_keys)
        if t is None or y is None:
            continue
        x_values.append(t)
        y_values.append(scale * y)
    return x_values, y_values


def padded_limits(values):
    finite_values = [value for value in values if math.isfinite(value)]
    if not finite_values:
        return None
    low = min(finite_values)
    high = max(finite_values)
    if low == high:
        padding = max(abs(low) * 0.05, 1.0e-6)
    else:
        padding = 0.05 * (high - low)
    return low - padding, high + padding


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "csv_path",
        nargs="?",
        default="/tmp/slider_positions.csv",
        help="CSV written by visualize_robobee_aeromechanical.",
    )
    parser.add_argument(
        "--window",
        type=float,
        default=0.05,
        help="Sim-time plot window in seconds.",
    )
    parser.add_argument(
        "--period",
        type=float,
        default=0.1,
        help="Refresh period in real seconds.",
    )
    args = parser.parse_args()

    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit(
            "matplotlib is required: python3 -m pip install matplotlib"
        ) from exc

    path = pathlib.Path(args.csv_path)
    plt.ion()
    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(10, 7))

    while True:
        if not path.exists():
            print(f"Waiting for {path}...")
            time.sleep(args.period)
            continue

        rows = read_rows(path)
        if not rows:
            time.sleep(args.period)
            continue

        times = [row_value(row, "time_s") for row in rows]
        times = [value for value in times if value is not None]
        if not times:
            time.sleep(args.period)
            continue

        t_max = times[-1]
        t_min = t_max - args.window
        rows_window = [
            row for row in rows
            if (row_value(row, "time_s") is not None and
                row_value(row, "time_s") >= t_min)
        ]

        position_axis = axes[0]
        position_axis.clear()
        position_values = []
        for keys, label, color, linestyle in [
            (("right_desired_displacement_m", "right_desired_m"),
             "right desired", "black", "--"),
            (("right_actual_displacement_m", "right_actual_m"),
             "right actual", "tab:blue", "-"),
            (("left_desired_displacement_m", "left_desired_m"),
             "left desired", "0.55", "--"),
            (("left_actual_displacement_m", "left_actual_m"),
             "left actual", "tab:orange", "-"),
        ]:
            x_values, y_values = xy_series(rows_window, keys, scale=1.0e3)
            if y_values:
                position_values.extend(y_values)
                position_axis.plot(x_values, y_values, color=color,
                                   linestyle=linestyle, label=label)

        limits = padded_limits(position_values)
        if limits is not None:
            position_axis.set_ylim(*limits)
        position_axis.set_ylabel("slider position [mm]")
        position_axis.grid(True, alpha=0.3)
        position_axis.legend(loc="upper right", ncol=2, fontsize="small")

        error_axis = axes[1]
        error_axis.clear()
        error_values = []
        for keys, label, color in [
            (("right_error_m",), "right actual - desired", "tab:blue"),
            (("left_error_m",), "left actual - desired", "tab:orange"),
        ]:
            x_values, y_values = xy_series(rows_window, keys, scale=1.0e6)
            if y_values:
                error_values.extend(y_values)
                error_axis.plot(x_values, y_values, color=color, label=label)

        limits = padded_limits(error_values)
        if limits is not None:
            error_axis.set_ylim(*limits)
        error_axis.set_ylabel("tracking error [um]")
        error_axis.grid(True, alpha=0.3)
        error_axis.legend(loc="upper right", ncol=2, fontsize="small")

        axes[-1].set_xlabel("simulation time [s]")
        axes[-1].set_xlim(t_min, t_max)
        fig.suptitle(f"Slider actual and desired positions from {path}")
        fig.tight_layout()
        plt.pause(0.001)
        time.sleep(args.period)


if __name__ == "__main__":
    main()
