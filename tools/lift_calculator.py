#!/usr/bin/env python3
"""
Figure-8-style lift calculation for Whitney & Wood passive flapping case.

Inputs:
  - aeromechanical_wing_constants.h
  - optional measured_kinematics.csv with columns:
        t_s, phi_deg, theta_deg, psi_deg
    or:
        t_ms, phi_deg, theta_deg, psi_deg

  - optional measured_lift.csv with columns:
        t_s or t_ms, lift_mg

If measured_kinematics.csv is not present, this script synthesizes a
simple 108 degree peak-to-peak sinusoidal flapping trajectory at 180 Hz.
That is useful for checking the model, but it is not the actual measured
Figure 8 trace.
"""

from __future__ import annotations

import re
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


# ----------------------------
# User settings
# ----------------------------

HEADER_PATH = Path("aeromechanical_wing_constants.h")
KINEMATICS_CSV = Path("measured_kinematics.csv")
MEASURED_LIFT_CSV = Path("measured_lift.csv")

F_WINGBEAT_HZ = 180.0
N_CYCLES = 2
N_SAMPLES = 2000

RHO_AIR = 1.225          # kg / m^3
G = 9.80665              # m / s^2

CL_MAX = 1.7             # Whitney & Wood use 1.7 for their calculations
CD_0 = 0.4
CD_MAX = 3.4

# Set this only if you know the wing mass and want to add a rigid-body
# inertial reaction estimate. The uploaded constants do not include wing mass.
WING_MASS_KG = None


# ----------------------------
# Read C++ wing constants
# ----------------------------

def read_wing_constants(header_path: Path) -> dict:
    text = header_path.read_text()

    marker = "kLeftWingAeromechanicalConstants = {"
    if marker not in text:
        raise ValueError(f"Could not find {marker!r} in {header_path}")

    body = text.split(marker, 1)[1].split("};", 1)[0]

    # Parse C/C++ floating point literals including scientific notation.
    nums = np.array(
        [float(x) for x in re.findall(
            r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?",
            body
        )],
        dtype=float,
    )

    if nums.size < 13:
        raise ValueError("Could not parse the scalar wing constants.")

    constants = {
        "R": nums[0],
        "area": nums[1],
        "cbar": nums[2],
        "span_axis_angle_rad": nums[3],
        "chord_axis_sign": nums[4],
        "x_r_hat": nums[5],
        "y_r_hat": nums[6],
        "r1_hat": nums[7],
        "r2_hat_squared": nums[8],
        "F_hat": nums[9],
        "Y_rd_hat": nums[10],
        "I_xy_am_hat": nums[11],
        "I_xx_am_hat": nums[12],
    }

    station_vals = nums[13:]
    if station_vals.size % 7 != 0:
        raise ValueError(
            f"Blade station data length {station_vals.size} is not divisible by 7."
        )

    stations = station_vals.reshape((-1, 7))
    constants["stations"] = stations

    return constants


# ----------------------------
# Kinematics
# ----------------------------

def load_or_synthesize_kinematics() -> dict:
    period = 1.0 / F_WINGBEAT_HZ
    t = np.linspace(0.0, N_CYCLES * period, N_SAMPLES)

    if KINEMATICS_CSV.exists():
        data = np.genfromtxt(KINEMATICS_CSV, delimiter=",", names=True)

        if "t_s" in data.dtype.names:
            t_in = data["t_s"]
        elif "t_ms" in data.dtype.names:
            t_in = data["t_ms"] * 1e-3
        else:
            raise ValueError("Kinematics CSV must contain t_s or t_ms.")

        phi = np.deg2rad(data["phi_deg"])
        theta = np.deg2rad(data["theta_deg"])
        psi = np.deg2rad(data["psi_deg"])

        return {
            "t": t_in,
            "phi": phi,
            "theta": theta,
            "psi": psi,
            "source": "measured_kinematics.csv",
        }

    # Synthetic Figure-8-like placeholder:
    # Whitney baseline case used 108 degrees peak-to-peak flapping.
    phi_amp = np.deg2rad(108.0 / 2.0)

    # A simple passive-rotation-like pitch waveform.
    # Replace with measured psi(t) for real Figure 8 reproduction.
    psi_amp = np.deg2rad(55.0)

    phi = phi_amp * np.sin(2.0 * np.pi * F_WINGBEAT_HZ * t)
    theta = np.zeros_like(t)
    psi = psi_amp * np.sin(2.0 * np.pi * F_WINGBEAT_HZ * t - np.pi / 2.0)

    return {
        "t": t,
        "phi": phi,
        "theta": theta,
        "psi": psi,
        "source": "synthetic 108 deg peak-to-peak sinusoid",
    }


def deriv(y: np.ndarray, t: np.ndarray) -> np.ndarray:
    return np.gradient(y, t, edge_order=2)


# ----------------------------
# Aerodynamics
# ----------------------------

def aerodynamic_coefficients(
    alpha: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    cl = CL_MAX * np.sin(2.0 * alpha)
    cd = 0.5 * (CD_MAX + CD_0) - 0.5 * (CD_MAX - CD_0) * np.cos(2.0 * alpha)
    cn = np.cos(alpha) * cl + np.sin(alpha) * cd
    return cl, cd, cn


def calculate_lift(constants: dict, kin: dict) -> dict:
    t = kin["t"]
    phi = kin["phi"]
    theta = kin["theta"]
    psi = kin["psi"]

    phi_dot = deriv(phi, t)
    theta_dot = deriv(theta, t)
    psi_dot = deriv(psi, t)

    # Whitney & Wood equation 2.8: wing angular velocity in wing frame.
    omega_x = psi_dot - phi_dot * np.sin(theta)
    omega_y = -phi_dot * np.cos(theta) * np.cos(psi) + theta_dot * np.sin(psi)
    omega_z = phi_dot * np.cos(theta) * np.sin(psi) + theta_dot * np.cos(psi)

    omega_x_dot = deriv(omega_x, t)
    omega_y_dot = deriv(omega_y, t)

    omega_h = np.sqrt(omega_y**2 + omega_z**2)
    alpha = np.arctan2(-omega_y, omega_z)

    cl, cd, cn = aerodynamic_coefficients(alpha)

    R = constants["R"]
    cbar = constants["cbar"]
    F_hat = constants["F_hat"]

    # Translational blade-element normal force magnitude.
    # This is the dominant aerodynamic lift-like term used for a Figure-8-style trace.
    f_aero_n = (
        0.5
        * RHO_AIR
        * omega_h**2
        * cn
        * cbar
        * R**3
        * F_hat
    )

    # Sign convention: make positive upward/lift-like.
    lift_aero = np.abs(f_aero_n)

    # Added-mass normal force from blade stations using the thin-section formula.
    stations = constants["stations"]
    r_m = stations[:, 0]
    chord_m = stations[:, 2]
    y_hinge_to_mid_chord_hat = stations[:, 6]

    dr = np.gradient(r_m)
    y_h = y_hinge_to_mid_chord_hat * cbar
    a = 0.5 * chord_m

    lambda_z = np.pi * RHO_AIR * a**2
    lambda_zomega = -np.pi * RHO_AIR * a**2 * y_h

    x_r = constants["x_r_hat"] * R
    r_from_hinge = r_m + x_r

    lift_added_mass = np.zeros_like(t)

    moment_aero_hinge = np.zeros_like(t)
    moment_added_mass_hinge = np.zeros_like(t)

    # Moment arm from hinge to section mid-chord / approximate center of pressure.
    # This is signed. If your C++ model has a more exact center-of-pressure offset,
    # replace this with that exact pitch-axis moment arm.
    moment_arm_hinge = y_h

    for i in range(t.size):
        # Wdot0 = r * (-omega_y_dot + omega_x * omega_z)
        wdot0 = r_from_hinge * (-omega_y_dot[i] + omega_x[i] * omega_z[i])

        # Section normal added-mass force per unit span:
        # Z0 = -lambda_z * Wdot0 - lambda_zomega * omega_x_dot
        z0 = -lambda_z * wdot0 - lambda_zomega * omega_x_dot[i]

        lift_added_mass[i] = np.sum(z0 * dr)

        # Added-mass hinge moment from section normal force.
        moment_added_mass_hinge[i] = np.sum(moment_arm_hinge * z0 * dr)

        # Translational aero hinge moment.
        #
        # Distributed normal force per unit span:
        # dF/dr = 0.5 rho omega_h^2 C_N c(r) r^2
        #
        # This recovers a distributed force/moment instead of using the
        # nondimensional F_hat collapse, because F_hat only gives total force.
        q_section = (
            0.5
            * RHO_AIR
            * omega_h[i]**2
            * cn[i]
            * chord_m
            * r_from_hinge**2
        )

        moment_aero_hinge[i] = np.sum(moment_arm_hinge * q_section * dr)

    # Optional rigid-wing inertial reaction. Figure 8 includes this,
    # but the uploaded constants do not provide wing mass.
    lift_inertial = np.zeros_like(t)
    if WING_MASS_KG is not None:
        # Placeholder: real implementation needs COM acceleration from measured geometry.
        # This is intentionally left zero unless mass/COM/inertia are supplied.
        pass

    lift_total = lift_aero + lift_added_mass + lift_inertial
    moment_total_hinge = moment_aero_hinge + moment_added_mass_hinge

    return {
        "t": t,
        "alpha": alpha,
        "phi": phi,
        "theta": theta,
        "psi": psi,
        "lift_aero_N": lift_aero,
        "lift_added_mass_N": lift_added_mass,
        "lift_inertial_N": lift_inertial,
        "lift_total_N": lift_total,
        "lift_total_mg": lift_total / G * 1e6,
        "lift_aero_mg": lift_aero / G * 1e6,
        "lift_added_mass_mg": lift_added_mass / G * 1e6,
        "moment_aero_hinge_Nm": moment_aero_hinge,
        "moment_added_mass_hinge_Nm": moment_added_mass_hinge,
        "moment_total_hinge_Nm": moment_total_hinge,
    }


# ----------------------------
# Plotting
# ----------------------------

def maybe_load_measured_lift():
    if not MEASURED_LIFT_CSV.exists():
        return None

    data = np.genfromtxt(MEASURED_LIFT_CSV, delimiter=",", names=True)

    if "t_s" in data.dtype.names:
        t = data["t_s"]
    elif "t_ms" in data.dtype.names:
        t = data["t_ms"] * 1e-3
    else:
        raise ValueError("Measured lift CSV must contain t_s or t_ms.")

    return t, data["lift_mg"]


def main() -> None:
    constants = read_wing_constants(HEADER_PATH)
    kin = load_or_synthesize_kinematics()
    out = calculate_lift(constants, kin)

    t_ms = out["t"] * 1e3

    print(f"Kinematics source: {kin['source']}")
    print(f"Wingbeat frequency: {F_WINGBEAT_HZ:.1f} Hz")
    print(f"Mean calculated lift: {np.mean(out['lift_total_mg']):.2f} mg")
    print(f"Peak calculated lift: {np.max(out['lift_total_mg']):.2f} mg")
    print(
        f"Peak absolute hinge moment: "
        f"{np.max(np.abs(out['moment_total_hinge_Nm'])):.3e} N m"
    )

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(8, 9.5))
    lift_axis, aero_force_axis, hinge_moment_axis = axes

    measured = maybe_load_measured_lift()
    if measured is not None:
        t_meas, lift_meas_mg = measured
        lift_axis.plot(t_meas * 1e3, lift_meas_mg, label="Lift_meas")

    lift_axis.plot(t_ms, out["lift_total_mg"], label="Lift_calc")
    lift_axis.plot(t_ms, out["lift_aero_mg"], "--", label="Aero term")
    lift_axis.plot(t_ms, out["lift_added_mass_mg"], ":", label="Added-mass term")
    lift_axis.set_ylabel("Lift (mg)")
    lift_axis.set_title(
        f"Figure-8-style lift calculation, f = {F_WINGBEAT_HZ:.0f} Hz"
    )
    lift_axis.grid(True, alpha=0.3)
    lift_axis.legend()

    aero_force_axis.plot(t_ms, out["lift_aero_N"])
    aero_force_axis.set_ylabel("lift_aero_N")
    aero_force_axis.grid(True, alpha=0.3)

    hinge_moment_axis.plot(
        t_ms,
        out["moment_total_hinge_Nm"],
        label="Total hinge moment",
    )
    hinge_moment_axis.plot(
        t_ms,
        out["moment_aero_hinge_Nm"],
        "--",
        label="Aero hinge moment",
    )
    hinge_moment_axis.plot(
        t_ms,
        out["moment_added_mass_hinge_Nm"],
        ":",
        label="Added-mass hinge moment",
    )

    hinge_moment_axis.set_xlabel("Time (ms)")
    hinge_moment_axis.set_ylabel("Moment at hinge (N m)")
    hinge_moment_axis.grid(True, alpha=0.3)
    hinge_moment_axis.legend()

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()