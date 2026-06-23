#!/usr/bin/env python3
"""Live plot aeromechanical moment CSV output from the simulator."""

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


def xy_series(rows, y_key):
    x_values = []
    y_values = []
    for row in rows:
        t = row_value(row, "time_s")
        y = row_value(row, y_key)
        if t is None or y is None:
            continue
        x_values.append(t)
        y_values.append(y)
    return x_values, y_values


def summed_xy_series(rows, y_keys):
    x_values = []
    y_values = []
    for row in rows:
        t = row_value(row, "time_s")
        if t is None:
            continue
        values = [row_value(row, key) for key in y_keys]
        if any(value is None for value in values):
            continue
        x_values.append(t)
        y_values.append(sum(values))
    return x_values, y_values


def expand_range(previous_range, values):
    finite_values = [
        value for value in values
        if value is not None and math.isfinite(value)
    ]
    if not finite_values:
        return previous_range

    current_range = (min(finite_values), max(finite_values))
    if previous_range is None:
        return current_range
    return (
        min(previous_range[0], current_range[0]),
        max(previous_range[1], current_range[1]),
    )


def padded_limits(value_range):
    low, high = value_range
    if low == high:
        padding = max(abs(low) * 0.05, 1.0e-12)
    else:
        padding = 0.05 * (high - low)
    return low - padding, high + padding


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "csv_path",
        nargs="?",
        default="/tmp/aeromechanical_moments.csv",
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
    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(11, 10))

    channels = [
        ("aero_Nm", "aero"),
        ("rot_Nm", "rot"),
        ("added_Nm", "added"),
        ("hinge_Nm", "hinge"),
        ("total_Nm", "total"),
        ("applied_Nm", "applied"),
    ]

    y_ranges = [None for _ in axes]
    last_t_max = None

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
      if last_t_max is not None and t_max < last_t_max:
          y_ranges = [None for _ in axes]
      last_t_max = t_max

      t_min = t_max - args.window
      rows_window = [
          row for row in rows
          if (row_value(row, "time_s") is not None and
              row_value(row, "time_s") >= t_min)
      ]

      vertical_force_axis = axes[0]
      vertical_force_axis.clear()
      vertical_force_values = [0.200e-3 * 9.80665, 0.300e-3 * 9.80665]
      vertical_force_axis.axhspan(
          vertical_force_values[0], vertical_force_values[1],
          color="tab:green", alpha=0.12, label="200-300 mgf")

      has_vertical_force = False
      for wing in ["left", "right"]:
          x_values, values = xy_series(rows_window, f"{wing}_force_z_N")
          if values:
              has_vertical_force = True
              vertical_force_values.extend(values)
              vertical_force_axis.plot(
                  x_values, values, label=f"{wing} force z")

      x_values, total_force_z = summed_xy_series(
          rows_window, ["left_force_z_N", "right_force_z_N"])
      if total_force_z:
          has_vertical_force = True
          vertical_force_values.extend(total_force_z)
          vertical_force_axis.plot(
              x_values, total_force_z, color="black", linewidth=1.6,
              label="total force z")

      y_ranges[0] = expand_range(y_ranges[0], vertical_force_values)
      if y_ranges[0] is not None:
          vertical_force_axis.set_ylim(*padded_limits(y_ranges[0]))

      vertical_force_axis.set_ylabel("world z force [N]")
      vertical_force_axis.grid(True, alpha=0.3)
      if has_vertical_force:
          vertical_force_axis.legend(
              loc="upper right", ncol=3, fontsize="small")
      else:
          vertical_force_axis.text(
              0.5, 0.5, "world z force not logged",
              transform=vertical_force_axis.transAxes,
              ha="center", va="center",
          )

      left_moment_axis = axes[1]
      left_moment_axis.clear()
      left_moment_values = []
      for suffix, label in channels:
          key = f"left_{suffix}"
          x_values, y_values = xy_series(rows_window, key)
          if y_values:
              left_moment_values.extend(y_values)
              left_moment_axis.plot(x_values, y_values, label=label)

      y_ranges[1] = expand_range(y_ranges[1], left_moment_values)
      if y_ranges[1] is not None:
          left_moment_axis.set_ylim(*padded_limits(y_ranges[1]))

      left_moment_axis.set_ylabel("left moment [N*m]")
      left_moment_axis.grid(True, alpha=0.3)
      left_moment_axis.legend(loc="upper right", ncol=3, fontsize="small")

      psi_axis = axes[2]
      psi_axis.clear()
      psi_values = []
      has_psi = False

      for wing in ["left", "right"]:
          key = f"{wing}_psi_rad"
          x_values, values = xy_series(rows_window, key)
          if values:
              has_psi = True
              psi_values.extend(values)
              psi_axis.plot(x_values, values, label=f"{wing} psi")

      y_ranges[2] = expand_range(y_ranges[2], psi_values)
      if y_ranges[2] is not None:
          psi_axis.set_ylim(*padded_limits(y_ranges[2]))

      psi_axis.set_ylabel("wing pitch psi [rad]")
      psi_axis.grid(True, alpha=0.3)
      if has_psi:
          psi_axis.legend(loc="upper right", ncol=2, fontsize="small")
      else:
          psi_axis.text(
              0.5, 0.5, "left_psi_rad/right_psi_rad not logged",
              transform=psi_axis.transAxes,
              ha="center", va="center",
          )

      force_z_axis = axes[3]
      force_z_axis.clear()
      has_force_z = False
      force_z_values = []
      for wing in ["left", "right"]:
          key = f"{wing}_force_z_N"
          x_values, force_z = xy_series(rows_window, key)
          if force_z:
              has_force_z = True
              force_z_values.extend(force_z)
              force_z_axis.plot(x_values, force_z, label=f"{wing} force z")

      y_ranges[3] = expand_range(y_ranges[3], force_z_values)
      if y_ranges[3] is not None:
          force_z_axis.set_ylim(*padded_limits(y_ranges[3]))

      force_z_axis.set_ylabel("world z force [N]")
      force_z_axis.grid(True, alpha=0.3)
      if has_force_z:
          force_z_axis.legend(loc="upper right", ncol=2, fontsize="small")
      else:
          force_z_axis.text(
              0.5, 0.5, "world z force not logged",
              transform=force_z_axis.transAxes,
              ha="center", va="center",
          )

      axes[-1].set_xlabel("simulation time [s]")
      axes[-1].set_xlim(t_min, t_max)
      fig.suptitle(f"Aeromechanical force_z_N, left moments, and wing pitch from {path}")
      fig.tight_layout()
      plt.pause(0.001)
      time.sleep(args.period)


if __name__ == "__main__":
    main()