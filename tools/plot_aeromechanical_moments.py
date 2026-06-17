#!/usr/bin/env python3
"""Live plot aeromechanical moment CSV output from the simulator."""

import argparse
import csv
import pathlib
import time


def read_rows(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def series(rows, key):
    return [float(row[key]) for row in rows if row.get(key)]


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
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(11, 9))
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

      t = series(rows, "time_s")
      if not t:
          time.sleep(args.period)
          continue
      t_max = t[-1]
      first = next((i for i, value in enumerate(t)
                    if value >= t_max - args.window), 0)
      rows_window = rows[first:]
      t_window = t[first:]

      for axis, wing in zip(axes[:2], ["left", "right"]):
          axis.clear()
          for suffix, label in channels:
              key = f"{wing}_{suffix}"
              axis.plot(t_window, series(rows_window, key), label=label)
          axis.set_ylabel(f"{wing} moment [N*m]")
          axis.grid(True, alpha=0.3)
          axis.legend(loc="upper right", ncol=3, fontsize="small")

      alpha_axis = axes[2]
      alpha_axis.clear()
      has_alpha = False
      for wing in ["left", "right"]:
          key = f"{wing}_alpha_rad"
          alpha = series(rows_window, key)
          if alpha:
              has_alpha = True
              alpha_axis.plot(t_window, alpha, label=f"{wing} alpha")
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

      axes[-1].set_xlabel("simulation time [s]")
      fig.suptitle(f"Aeromechanical moments from {path}")
      fig.tight_layout()
      plt.pause(0.001)
      time.sleep(args.period)


if __name__ == "__main__":
    main()
