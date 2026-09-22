#!/usr/bin/env python3
"""Live plot of the wing stroke angles from the simulator's slider CSV.

Reads the same /tmp/slider_positions.csv that plot_slider_positions.py uses
(written by both the standalone app and the Simulink TCP server) and plots the
right_stroke_angle_rad / left_stroke_angle_rad columns in degrees, with the
commanded actuator voltages underneath for phase context.

The stroke angle is the transmission output joint angle (revolute_1_4 right,
revolute_1_3 left) relative to its CAD rest angle. The two joints have mirrored
positive directions in the URDF, so symmetric flapping appears anti-phase in the
raw signals; pass --flip-right to negate the right wing so the two overlay.

If matplotlib error:
    source .venv/bin/activate
    python tools/plot_stroke_angles.py
"""
import argparse
import math
import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from plot_slider_positions import padded_limits, read_rows, row_value  # noqa: E402

RIGHT_COLOR = "#2a78d6"
LEFT_COLOR = "#eb6834"
VOLTAGE_RIGHT_COLOR = "0.25"
VOLTAGE_LEFT_COLOR = "0.6"


def series(rows, key, scale=1.0):
    x_values = []
    y_values = []
    for row in rows:
        t = row_value(row, "time_s")
        y = row_value(row, key)
        if t is None or y is None:
            continue
        x_values.append(t)
        y_values.append(scale * y)
    return x_values, y_values


def draw(fig, axes, rows_window, t_min, t_max, path, flip_right, show_side):
    angle_axis, voltage_axis = axes
    angle_axis.clear()
    angle_values = []
    right_scale = -math.degrees(1.0) if flip_right else math.degrees(1.0)
    # Draw left first, then right on top; when flipped the two traces coincide
    # for symmetric flapping, so the right one is dashed to stay visible.
    for key, label, color, scale, linestyle in [
        ("left_stroke_angle_rad", "left", LEFT_COLOR, math.degrees(1.0), "-"),
        ("right_stroke_angle_rad",
         "right" + (" (sign flipped)" if flip_right else ""), RIGHT_COLOR,
         right_scale, "-" if flip_right else "-"),
    ]:
        if not show_side(label):
            continue
        x_values, y_values = series(rows_window, key, scale)
        if y_values:
            angle_values.extend(y_values)
            angle_axis.plot(x_values, y_values, color=color, linewidth=1.5,
                            linestyle=linestyle, label=label)
    limits = padded_limits(angle_values)
    if limits is not None:
        angle_axis.set_ylim(*limits)
    angle_axis.axhline(0.0, color="0.8", linewidth=0.8, zorder=0)
    angle_axis.set_ylabel("stroke angle [deg]")
    angle_axis.grid(True, alpha=0.3)
    if angle_values:
        angle_axis.legend(loc="upper right", ncol=2, fontsize="small")
    else:
        angle_axis.text(0.5, 0.5,
                        "no stroke angle columns in CSV\n"
                        "(rebuild the simulator; columns were added 2026-09-16)",
                        ha="center", va="center", transform=angle_axis.transAxes)

    voltage_axis.clear()
    voltage_values = []
    for key, label, color, linestyle in [
        ("right_voltage_v", "right", VOLTAGE_RIGHT_COLOR, "-"),
        ("left_voltage_v", "left", VOLTAGE_LEFT_COLOR, "--"),
    ]:
        if not show_side(label):
            continue
        x_values, y_values = series(rows_window, key)
        if y_values:
            voltage_values.extend(y_values)
            voltage_axis.plot(x_values, y_values, color=color,
                              linestyle=linestyle, linewidth=1.2, label=label)
    limits = padded_limits(voltage_values)
    if limits is not None:
        voltage_axis.set_ylim(*limits)
    voltage_axis.set_ylabel("actuator voltage [V]")
    voltage_axis.set_xlabel("simulation time [s]")
    voltage_axis.grid(True, alpha=0.3)
    if voltage_values:
        voltage_axis.legend(loc="upper right", ncol=2, fontsize="small")
    voltage_axis.set_xlim(t_min, t_max)
    fig.suptitle(f"Wing stroke angles from {path}")
    fig.tight_layout()


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "csv_path",
        nargs="?",
        default="/tmp/slider_positions.csv",
        help="CSV written by visualize_robobee_aeromechanical or the TCP server.",
    )
    parser.add_argument("--window", type=float, default=0.05,
                        help="Sim-time plot window in seconds.")
    parser.add_argument("--period", type=float, default=0.1,
                        help="Refresh period in real seconds.")
    parser.add_argument("--side", choices=("left", "right", "both"),
                        default="both", help="Which wing(s) to plot.")
    parser.add_argument("--flip-right", action="store_true",
                        help="Negate the right stroke angle so symmetric "
                             "flapping overlays the left wing.")
    parser.add_argument("--once", metavar="PNG",
                        help="Render the current window once to this PNG and exit "
                             "(no live loop, no display).")
    args = parser.parse_args()
    show_side = lambda label: args.side == "both" or label.startswith(args.side)

    try:
        import matplotlib
        if args.once:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit(
            "matplotlib is required: python3 -m pip install matplotlib"
        ) from exc

    path = pathlib.Path(args.csv_path)
    if not args.once:
        plt.ion()
    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(10, 7))

    while True:
        if not path.exists():
            if args.once:
                raise SystemExit(f"{path} does not exist")
            print(f"Waiting for {path}...")
            time.sleep(args.period)
            continue

        rows = read_rows(path)
        times = [row_value(row, "time_s") for row in rows]
        times = [value for value in times if value is not None]
        if not times:
            if args.once:
                raise SystemExit(f"{path} has no rows")
            time.sleep(args.period)
            continue

        t_max = times[-1]
        t_min = t_max - args.window
        rows_window = [
            row for row in rows
            if (row_value(row, "time_s") is not None and
                row_value(row, "time_s") >= t_min)
        ]
        draw(fig, axes, rows_window, t_min, t_max, path, args.flip_right,
             show_side)

        if args.once:
            fig.savefig(args.once, dpi=120)
            print(f"wrote {args.once}")
            return
        plt.pause(0.001)
        time.sleep(args.period)


if __name__ == "__main__":
    main()
