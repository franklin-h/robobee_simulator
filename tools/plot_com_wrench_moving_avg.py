#!/usr/bin/env python3
"""Plot a trailing-window moving average of the RoboBee COM wrench.

Variant of plot_com_wrench.py. Instead of plotting the raw signals in stacked
subplots, this plots a trailing-window moving average of each channel: at every
timestep i the plotted value is the mean of the last `window` samples ending at
i. Unlike a cumulative average, this tracks the value the signal settles to, so
the curve flattens where the torque converges and the steady-state reference
line lands on that flat region. Each channel (z-axis thrust, roll/pitch/yaw
torque) is drawn in its own separate figure rather than as a subplot.

The wrench is a periodic flapping signal (default 155 Hz) riding on the DC
offset you actually care about. To recover that offset the trailing window MUST
span a whole number of flap cycles, otherwise the leftover partial cycle keeps
the moving average wobbling and biases the steady-state line. By default the
window is therefore computed from the data's sample rate and the flap frequency
as `round(cycles / (flap_hz * dt))` samples, so it is an integer number of flap
cycles regardless of the logging rate. `--window` overrides this with an
explicit sample count; `--start` is likewise expressed in flap cycles.

Usage:
  python3 tools/plot_com_wrench_moving_avg.py                # one-shot
  python3 tools/plot_com_wrench_moving_avg.py --live         # refresh continuously
  python3 tools/plot_com_wrench_moving_avg.py --cycles 30    # avg over 30 flap cycles
  python3 tools/plot_com_wrench_moving_avg.py --flap-hz 160  # flap frequency [Hz]
  python3 tools/plot_com_wrench_moving_avg.py --window 300   # force raw sample count
"""

import argparse
import csv
import math
import pathlib
import sys
import time


# CSV is logged in SI (thrust N, torque N·m). `scale` converts each channel to
# its display unit: thrust N -> mN (x1e3), torque N·m -> mN·mm (x1e6, since
# 1 N·m = 1e3 mN * 1e3 mm). Plotted values, reference lines and printed averages
# all use these display units.
CHANNELS = [
    ("thrust_z_N", "z-axis thrust", "mN", "tab:blue", 1e3),
    ("roll_torque_Nm", "roll torque", "mN·mm", "tab:red", 1e6),
    ("pitch_torque_Nm", "pitch torque", "mN·mm", "tab:green", 1e6),
    ("yaw_torque_Nm", "yaw torque", "mN·mm", "tab:purple", 1e6),
    # Controller axes: +x forward, +y left. Absent in CSVs recorded before
    # force_x_N/force_y_N were added; these figures then show "(no data)".
    ("force_x_N", "x-axis (fwd) force", "mN", "tab:orange", 1e3),
    ("force_y_N", "y-axis (left) force", "mN", "tab:brown", 1e3),
]

# Manual y-axis limits per channel, expressed in the DISPLAY units above
# (mN / mN·mm). Set (low, high) to override auto-scaling; leave as None to
# auto-scale that channel from its plotted running average.
YLIMS = {
    "thrust_z_N": None,
    "roll_torque_Nm": None,
    "pitch_torque_Nm": (-0.5, 0.8),
    "yaw_torque_Nm": None,
    "force_x_N": None,
    "force_y_N": None,
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


def estimate_dt(rows):
    """Median sample spacing (s) of the time_s column, or None if unknown."""
    times = sorted(
        t for t in (parse_float(row.get("time_s")) for row in rows)
        if t is not None
    )
    diffs = sorted(b - a for a, b in zip(times, times[1:]) if b > a)
    if not diffs:
        return None
    return diffs[len(diffs) // 2]


def samples_per_cycle(rows, flap_hz):
    """How many logged samples make up one flap cycle, or None if unknown."""
    dt = estimate_dt(rows)
    if dt is None or flap_hz <= 0.0:
        return None
    return 1.0 / (flap_hz * dt)


def resolve_window(rows, window_arg, cycles, flap_hz):
    """Trailing window in samples spanning an integer number of flap cycles.

    An explicit --window wins. Otherwise the window is
    round(cycles * samples_per_cycle) so the periodic flap component averages
    out and only the DC offset (the value we want) survives.
    """
    if window_arg is not None:
        return max(1, window_arg)
    per_cycle = samples_per_cycle(rows, flap_hz)
    if per_cycle is None:
        return 50
    return max(1, round(cycles * per_cycle))


def resolve_start(rows, start_arg, transient_cycles, flap_hz):
    """First plotted sample index. --start wins; else drop transient_cycles."""
    if start_arg is not None:
        return max(0, start_arg)
    per_cycle = samples_per_cycle(rows, flap_hz)
    if per_cycle is None:
        return 200
    return max(0, round(transient_cycles * per_cycle))


def trailing_average(x_values, y_values, start, window):
    """Return (x, avg) where avg[i] = mean of the trailing `window` samples.

    Only samples from index `start` onward are emitted. Each plotted point is
    the mean of samples (i - window + 1 .. i), clamped so the window never
    reaches earlier than `start` (i.e. the window grows from 1 up to `window`
    over the first `window` plotted points, then slides). This tracks the value
    the signal settles to rather than accumulating the whole history.
    """
    x_out = []
    avg_out = []
    for i in range(start, len(x_values)):
        lo = max(start, i - window + 1)
        segment = y_values[lo:i + 1]
        x_out.append(x_values[i])
        avg_out.append(sum(segment) / len(segment))
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


def compute_averages(rows, window):
    """Steady-state average of each channel: mean of the last `window` samples.

    This matches the endpoint of the trailing-window curve, so the reference
    line coincides with where the signal settles.
    """
    averages = {}
    for key, _label, _unit, _color, scale in CHANNELS:
        _x, y = xy_series(rows, key)
        avg = average(y[-window:])
        averages[key] = None if avg is None else scale * avg
    return averages


def print_averages(averages, sample_count, span, window, per_cycle=None):
    print("=" * 52)
    if per_cycle:
        print(f"Steady-state avg over last {window} of {sample_count} samples "
              f"(~{window / per_cycle:.2f} flap cycles, {span:.4f} s):")
    else:
        print(f"Steady-state avg over last {window} of {sample_count} samples "
              f"({span:.4f} s of sim time):")
    for key, label, unit, _color, _scale in CHANNELS:
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


def draw(figures, rows, averages, start, window):
    for (fig, axis), (key, label, unit, color, scale) in zip(figures, CHANNELS):
        axis.clear()
        x, y = xy_series(rows, key)
        y = [scale * v for v in y]
        x_avg, y_avg = trailing_average(x, y, start, window)
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
                         label=f"steady-state avg = {mean:+.4e} {unit}")
            axis.legend(loc="upper right", fontsize="small")
        axis.set_ylabel(f"{window}-sample avg {label} [{unit}]")
        axis.set_xlabel("simulation time [s]")
        axis.grid(True, alpha=0.3)
        axis.set_title(f"Trailing {window}-sample average of {label} "
                       f"about the RoboBee COM")
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
        default=None,
        help="First sample index plotted (drops the initial transient). "
             "Default: drop --transient-cycles flap cycles.",
    )
    parser.add_argument(
        "--window",
        type=int,
        default=None,
        help="Trailing window size (samples). Default: an integer number of "
             "flap cycles derived from --cycles, --flap-hz and the sample rate.",
    )
    parser.add_argument(
        "--cycles",
        type=float,
        default=30.0,
        help="Number of flap cycles the trailing window should span when "
             "--window is not given.",
    )
    parser.add_argument(
        "--flap-hz",
        type=float,
        default=155.0,
        help="Flapping frequency [Hz] used to size the window in whole cycles.",
    )
    parser.add_argument(
        "--transient-cycles",
        type=float,
        default=6.0,
        help="Flap cycles of initial transient to drop when --start is unset.",
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
        window = resolve_window(rows, args.window, args.cycles, args.flap_hz)
        per_cycle = samples_per_cycle(rows, args.flap_hz)
        print_averages(compute_averages(rows, window), len(rows),
                       time_span(rows), window, per_cycle)
        return

    try:
        import matplotlib.pyplot as plt
        
    except ImportError as exc:
        raise SystemExit(
            "matplotlib is required: python3 -m pip install matplotlib"
        ) from exc

    # One separate figure per channel instead of stacked subplots.
    figures = []
    for _key, label, _unit, _color, _scale in CHANNELS:
        fig, axis = plt.subplots(1, 1, figsize=(9, 4), num=label)
        figures.append((fig, axis))

    if not args.live:
        if not path.exists():
            raise SystemExit(f"{path} does not exist yet.")
        rows = window_rows(read_rows(path), args.last)
        if not rows:
            raise SystemExit(f"No usable rows in {path}.")
        window = resolve_window(rows, args.window, args.cycles, args.flap_hz)
        start = resolve_start(rows, args.start, args.transient_cycles, args.flap_hz)
        per_cycle = samples_per_cycle(rows, args.flap_hz)
        averages = compute_averages(rows, window)
        print_averages(averages, len(rows), time_span(rows), window, per_cycle)
        draw(figures, rows, averages, start, window)
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
        window = resolve_window(rows, args.window, args.cycles, args.flap_hz)
        start = resolve_start(rows, args.start, args.transient_cycles, args.flap_hz)
        per_cycle = samples_per_cycle(rows, args.flap_hz)
        averages = compute_averages(rows, window)
        print_averages(averages, len(rows), time_span(rows), window, per_cycle)
        draw(figures, rows, averages, start, window)
        plt.pause(0.001)
        time.sleep(args.period)


if __name__ == "__main__":
    main()
