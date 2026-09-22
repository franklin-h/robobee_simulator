#!/usr/bin/env python3
"""Stroke-angle frequency response (Bode plot) of the RoboBee actuator +
transmission + wing, measured by driving the standalone Drake app at a series
of frequencies and fitting the steady-state stroke angle.

For each (Vpp, frequency) pair this script runs

    visualize_robobee_aeromechanical --drive_frequency=F --duration=T
        --realtime_rate=0 --voltage_peak_to_peak=Vpp --slider_log=<file>

with the root body welded (the paper's clamped-body measurement), discards the
first --settle-cycles wingbeats, and least-squares fits
    y(t) = c + a sin(w t) + b cos(w t)
to the logged right stroke angle, slider displacement and actuator voltage over
the last --measure-cycles wingbeats. Amplitude ratio and phase of stroke relative
to voltage give one Bode point. Aerodynamic drag is quadratic in stroke rate, so
the response is amplitude dependent: run several --vpp. The console summary also
prints the natural frequency of the lumped model from Steinmeyer et al. (ICRA
2019, Table I) for comparison; it is not drawn on the plot.

Typical use (from the repo root, after `bazel build //apps:visualize_robobee_aeromechanical`):

    source .venv/bin/activate
    python3 tools/stroke_frequency_sweep.py --frequencies 40:400:5 --vpp 50,100,200

Outputs <out-dir>/stroke_sweep.csv and <out-dir>/stroke_bode.png. To redraw the
plot from an earlier sweep without re-simulating:

    python3 tools/stroke_frequency_sweep.py --from-csv /tmp/stroke_sweep/stroke_sweep.csv
"""
import argparse
import concurrent.futures
import csv
import math
import pathlib
import subprocess
import sys

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
DEFAULT_BINARY = REPO_ROOT / "bazel-bin" / "apps" / "visualize_robobee_aeromechanical"
ACTUATOR_REST_VOLTAGE_V = 100.0

# Steinmeyer et al. 2019, Table I, slider-side equivalent mass and stiffness;
# used only for the natural-frequency line in the console summary.
LUMPED = {
    "m_eq_kg": 0.388e-3,
    "k_eq_N_per_m": 500.4,
}

# Fixed categorical order (dataviz skill default palette, light surface).
SERIES_COLORS = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#4a3aa7"]


def parse_list(text, kind=float):
    """'a:b:step' -> inclusive range; 'x,y,z' -> list."""
    if ":" in text:
        parts = [kind(p) for p in text.split(":")]
        if len(parts) != 3:
            raise argparse.ArgumentTypeError("range must be start:stop:step")
        start, stop, step = parts
        if step <= 0:
            raise argparse.ArgumentTypeError("range step must be positive")
        values = []
        n = int(math.floor((stop - start) / step + 1e-9))
        for i in range(n + 1):
            values.append(start + i * step)
        return values
    return [kind(p) for p in text.split(",") if p.strip()]


def read_csv_rows(path):
    csv.field_size_limit(sys.maxsize)
    rows = []
    with path.open(newline="") as stream:
        for row in csv.DictReader(stream):
            if row and all(v not in (None, "") for v in row.values()):
                try:
                    rows.append({k: float(v) for k, v in row.items()})
                except ValueError:
                    continue
    return rows


def fit_sinusoid(t, y, omega):
    """Least-squares fit y = c + a sin(wt) + b cos(wt). Returns amplitude,
    phase (rad, y ~ A sin(wt + phase)), offset, and residual/fundamental rms."""
    import numpy as np

    t = np.asarray(t)
    y = np.asarray(y)
    design = np.column_stack([np.ones_like(t), np.sin(omega * t), np.cos(omega * t)])
    coef, *_ = np.linalg.lstsq(design, y, rcond=None)
    c, a, b = coef
    amplitude = math.hypot(a, b)
    phase = math.atan2(b, a)
    residual = y - design @ coef
    residual_rms = float(np.sqrt(np.mean(residual**2)))
    fundamental_rms = amplitude / math.sqrt(2.0)
    ratio = residual_rms / fundamental_rms if fundamental_rms > 0 else float("nan")
    return amplitude, phase, float(c), ratio


def wrap_deg(angle_deg):
    return (angle_deg + 180.0) % 360.0 - 180.0


def run_one(binary, freq_hz, vpp, bias_v, settle_cycles, measure_cycles,
            out_dir, keep_runs=False):
    total_cycles = settle_cycles + measure_cycles
    duration_s = total_cycles / freq_hz
    log_path = out_dir / f"slider_f{freq_hz:g}_vpp{vpp:g}.csv"
    cmd = [
        str(binary),
        f"--drive_frequency={freq_hz}",
        f"--duration={duration_s}",
        "--realtime_rate=0",
        f"--voltage_peak_to_peak={vpp}",
        f"--voltage_bias={bias_v}",
        f"--slider_log={log_path}",
    ]
    subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL,
                   stderr=subprocess.PIPE, text=True)
    rows = read_csv_rows(log_path)
    if not keep_runs:
        # Each per-run log is ~1 MB; a full sweep leaves hundreds of them.
        log_path.unlink(missing_ok=True)
    t_start = settle_cycles / freq_hz
    rows = [r for r in rows if r["time_s"] >= t_start]
    if len(rows) < 20:
        raise RuntimeError(f"too few steady-state rows for f={freq_hz}, vpp={vpp}")
    omega = 2.0 * math.pi * freq_hz
    t = [r["time_s"] for r in rows]
    volt = [r["right_voltage_v"] - ACTUATOR_REST_VOLTAGE_V for r in rows]
    stroke_r = [r["right_stroke_angle_rad"] for r in rows]
    stroke_l = [r["left_stroke_angle_rad"] for r in rows]
    slider_r = [r["right_actual_displacement_m"] for r in rows]

    # The URDF's right and left stroke joints have mirrored positive directions,
    # and the right one runs opposite to positive slider displacement. Orient the
    # stroke angle along the slider direction (sign of the stroke-vs-slider
    # slope) so that the low-frequency phase reads ~0 deg, as in the paper.
    import numpy as np
    x_arr = np.asarray(slider_r)
    s_arr = np.asarray(stroke_r)
    slope = float(np.dot(x_arr - x_arr.mean(), s_arr - s_arr.mean()) /
                  max(np.dot(x_arr - x_arr.mean(), x_arr - x_arr.mean()), 1e-30))
    sign = -1.0 if slope < 0 else 1.0
    stroke_r = [sign * s for s in stroke_r]

    v_amp, v_phase, _, _ = fit_sinusoid(t, volt, omega)
    sr_amp, sr_phase, sr_off, sr_resid = fit_sinusoid(t, stroke_r, omega)
    sl_amp, _, _, _ = fit_sinusoid(t, stroke_l, omega)
    x_amp, x_phase, x_off, _ = fit_sinusoid(t, slider_r, omega)

    return {
        "stroke_sign_vs_slider": sign,
        "vpp_v": vpp,
        "freq_hz": freq_hz,
        "voltage_amp_v": v_amp,
        "right_stroke_amp_rad": sr_amp,
        "left_stroke_amp_rad": sl_amp,
        "right_stroke_amp_deg": math.degrees(sr_amp),
        "right_stroke_offset_deg": math.degrees(sr_off),
        "stroke_gain_deg_per_v": math.degrees(sr_amp) / v_amp,
        "stroke_phase_deg": wrap_deg(math.degrees(sr_phase - v_phase)),
        "right_slider_amp_m": x_amp,
        "slider_phase_deg": wrap_deg(math.degrees(x_phase - v_phase)),
        "transmission_ratio_rad_per_m": sr_amp / x_amp if x_amp > 0 else float("nan"),
        "stroke_residual_ratio": sr_resid,
        "measure_rows": len(rows),
    }


def write_results(path, results):
    keys = list(results[0].keys())
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=keys)
        writer.writeheader()
        for r in results:
            writer.writerow(r)


def plot(results, out_path, show):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    vpps = sorted({r["vpp_v"] for r in results})

    fig, (ax_mag, ax_phase) = plt.subplots(2, 1, sharex=True, figsize=(9, 7))
    for i, vpp in enumerate(vpps):
        color = SERIES_COLORS[i % len(SERIES_COLORS)]
        pts = sorted((r for r in results if r["vpp_v"] == vpp),
                     key=lambda r: r["freq_hz"])
        f = [r["freq_hz"] for r in pts]
        # Gain in rad/V, plotted with a 1e-3 axis multiplier (i.e. mrad/V).
        gain_mrad_per_v = [1e3 * math.radians(r["stroke_gain_deg_per_v"]) for r in pts]
        ax_mag.plot(f, gain_mrad_per_v, color=color,
                    linewidth=2, marker="o", markersize=4,
                    label=f"{vpp:g} Vpp")
        # Unwrap along frequency so a crossing of -180 deg does not jump to +180.
        import numpy as np
        phase = np.degrees(np.unwrap(np.radians([r["stroke_phase_deg"] for r in pts])))
        ax_phase.plot(f, phase, color=color, linewidth=2, marker="o", markersize=4)

    ax_mag.set_ylabel(r"stroke gain [$\times 10^{-3}$ rad/V]")
    ax_mag.grid(True, alpha=0.25)
    if len(vpps) > 1:
        ax_mag.legend(loc="best", fontsize="small", title="actuator drive")
    ax_phase.set_ylabel("stroke phase rel. voltage [deg]")
    ax_phase.set_xlabel("drive frequency [Hz]")
    ax_phase.grid(True, alpha=0.25)
    ax_phase.set_ylim(-270, 30)
    fig.suptitle("Wing stroke angle frequency response (root welded)")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    print(f"wrote {out_path}")
    if show:
        plt.show()


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--binary", type=pathlib.Path, default=DEFAULT_BINARY,
                        help="standalone (welded) visualize_robobee_aeromechanical binary")
    parser.add_argument("--frequencies", default="40:400:10",
                        help="Hz, as start:stop:step or comma list")
    parser.add_argument("--vpp", default="200",
                        help="actuator-side peak-to-peak volts, comma list")
    parser.add_argument("--bias", type=float, default=200.0,
                        help="bias/upper-rail voltage passed through for logging")
    parser.add_argument("--settle-cycles", type=int, default=30,
                        help="wingbeats discarded before fitting")
    parser.add_argument("--measure-cycles", type=int, default=10,
                        help="wingbeats used for the sinusoid fit")
    parser.add_argument("--jobs", type=int, default=6,
                        help="parallel simulator processes (each writes its own slider CSV; "
                             "the other /tmp logs are clobbered and should be ignored)")
    parser.add_argument("--out-dir", type=pathlib.Path,
                        default=pathlib.Path("/tmp/stroke_sweep"))
    parser.add_argument("--no-plot", action="store_true")
    parser.add_argument("--no-show", action="store_true",
                        help="save the PNG without opening a window")
    parser.add_argument("--keep-runs", action="store_true",
                        help="keep each run's slider CSV in out-dir (default: delete "
                             "after fitting to save disk space)")
    parser.add_argument("--from-csv", type=pathlib.Path,
                        help="redraw the plot from an existing stroke_sweep.csv "
                             "instead of running the simulator")
    args = parser.parse_args()

    if args.from_csv is not None:
        results = read_csv_rows(args.from_csv)
        if not results:
            raise SystemExit(f"no rows in {args.from_csv}")
        args.out_dir.mkdir(parents=True, exist_ok=True)
        plot(results, args.out_dir / "stroke_bode.png", show=not args.no_show)
        return

    if not args.binary.exists():
        raise SystemExit(f"binary not found: {args.binary}\n"
                         "build it with: bazel build //apps:visualize_robobee_aeromechanical")
    freqs = parse_list(args.frequencies)
    vpps = parse_list(args.vpp)
    args.out_dir.mkdir(parents=True, exist_ok=True)

    jobs = [(f, v) for v in vpps for f in freqs]
    print(f"{len(jobs)} runs: {len(freqs)} frequencies x {len(vpps)} amplitudes, "
          f"{args.settle_cycles}+{args.measure_cycles} cycles each")
    results = []

    def task(job):
        f, v = job
        r = run_one(args.binary, f, v, args.bias, args.settle_cycles,
                    args.measure_cycles, args.out_dir, args.keep_runs)
        print(f"  f={f:7.2f} Hz  Vpp={v:6.1f}  stroke amp={r['right_stroke_amp_deg']:6.2f} deg  "
              f"gain={r['stroke_gain_deg_per_v']:.4f} deg/V  phase={r['stroke_phase_deg']:7.1f} deg  "
              f"T={r['transmission_ratio_rad_per_m']:.0f} rad/m  resid={r['stroke_residual_ratio']:.3f}",
              flush=True)
        return r

    if args.jobs > 1:
        with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as pool:
            results = list(pool.map(task, jobs))
    else:
        results = [task(j) for j in jobs]

    results.sort(key=lambda r: (r["vpp_v"], r["freq_hz"]))
    csv_path = args.out_dir / "stroke_sweep.csv"
    write_results(csv_path, results)
    print(f"wrote {csv_path}")

    peak = max(results, key=lambda r: r["stroke_gain_deg_per_v"])
    print(f"peak gain {peak['stroke_gain_deg_per_v']:.4f} deg/V at {peak['freq_hz']:g} Hz "
          f"({peak['vpp_v']:g} Vpp); lumped-model f_n = "
          f"{math.sqrt(LUMPED['k_eq_N_per_m'] / LUMPED['m_eq_kg']) / (2 * math.pi):.1f} Hz")

    if not args.no_plot:
        plot(results, args.out_dir / "stroke_bode.png", show=not args.no_show)


if __name__ == "__main__":
    main()
