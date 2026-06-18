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
    fig, axes = plt.subplots(5, 1, sharex=True, figsize=(11, 12))
    channels = [
        ("aero_Nm", "aero"),
        ("rot_Nm", "rot"),
        ("added_Nm", "added"),
        ("hinge_Nm", "hinge"),
        ("total_Nm", "total"),
        ("applied_Nm", "applied"),
    ]

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
      rows_window = [
          row for row in rows
          if (row_value(row, "time_s") is not None and
              row_value(row, "time_s") >= t_max - args.window)
      ]

      for axis, wing in zip(axes[:2], ["left", "right"]):
          axis.clear()
          for suffix, label in channels:
              key = f"{wing}_{suffix}"
              x_values, y_values = xy_series(rows_window, key)
              if y_values:
                  axis.plot(x_values, y_values, label=label)
          axis.set_ylabel(f"{wing} moment [N*m]")
          axis.grid(True, alpha=0.3)
          axis.legend(loc="upper right", ncol=3, fontsize="small")

      alpha_axis = axes[2]
      alpha_axis.clear()
      has_alpha = False
      for wing in ["left", "right"]:
          key = f"{wing}_alpha_rad"
          x_values, alpha = xy_series(rows_window, key)
          if alpha:
              has_alpha = True
              alpha_axis.plot(x_values, alpha, label=f"{wing} alpha")
      alpha_axis.set_ylabel("angle of attack [rad]")
      alpha_axis.grid(True, alpha=0.3)
      if has_alpha:
          alpha_axis.legend(loc="upper right", ncol=2, fontsize="small")
      else:
          alpha_axis.text(
              0.5, 0.5, "angle of attack not logged",
              transform=alpha_axis.transAxes,
              ha="center", va="center",
          )

      vertical_force_axis = axes[3]
      vertical_force_axis.clear()
      has_vertical_force = False
      for wing in ["left", "right"]:
          x_values, values = xy_series(rows_window, f"{wing}_force_z_N")
          if values:
              has_vertical_force = True
              vertical_force_axis.plot(
                  x_values, values, label=f"{wing} force z")
      vertical_force_axis.set_ylabel("world z force [N]")
      vertical_force_axis.grid(True, alpha=0.3)
      if has_vertical_force:
          vertical_force_axis.legend(
              loc="upper right", ncol=2, fontsize="small")
      else:
          vertical_force_axis.text(
              0.5, 0.5, "world z force not logged",
              transform=vertical_force_axis.transAxes,
              ha="center", va="center",
          )

      drag_axis = axes[4]
      drag_axis.clear()
      has_drag = False
      for wing in ["left", "right"]:
          x_values, values = xy_series(rows_window, f"{wing}_drag_N")
          if values:
              has_drag = True
              drag_axis.plot(x_values, values, label=f"{wing} drag")
      drag_axis.set_ylabel("drag [N]")
      drag_axis.grid(True, alpha=0.3)
      if has_drag:
          drag_axis.legend(loc="upper right", ncol=2, fontsize="small")
      else:
          drag_axis.text(
              0.5, 0.5, "drag force not logged",
              transform=drag_axis.transAxes,
              ha="center", va="center",
          )

      axes[-1].set_xlabel("simulation time [s]")
      fig.suptitle(f"Aeromechanical moments from {path}")
      fig.tight_layout()
      plt.pause(0.001)
      time.sleep(args.period)


if __name__ == "__main__":
    main()
