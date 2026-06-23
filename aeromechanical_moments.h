#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "aeromechanical_wing_constants.h"

namespace robobee {

// Implements the scalar wing-pitch moment model documented in
// README/aeromechanical_model.tex. The comments below cite the corresponding
// LaTeX subsections so the code can be checked against the derivation.
constexpr double kAeromechanicalPi = 3.14159265358979323846;

// Model constants used by the equations in the .tex file:
// - "Compute Aerodynamic Coefficients" defines rho, C_L,max, C_D,max, C_D,0.
// - "Compute aerodynamic damping moment" uses C_rd.
// - "Compute passive hinge restoring moment" uses the Kapton hinge geometry.
// The max_abs_* values are simulator guards rather than terms from the paper.
struct AeromechanicalModelParameters {
  double air_density_kg_m3{1.225};
  double cl_max{1.8};
  double cd_max{3.4};
  double cd_0{0.4};
  double rotational_damping_coefficient{2.0};

  double kapton_youngs_modulus_pa{2.5e9};
  double hinge_thickness_m{10.0e-6};
  double hinge_width_m{2.7e-3};
  double hinge_length_m{0.10e-3};

  double max_abs_applied_total_moment_Nm{2.0e-05};
  double max_abs_total_aerodynamic_force_N{1.0e-01};
};

// Local flow at one blade station, resolved in the wing chord-normal plane.
// This is the code form of README/aeromechanical_model.tex, subsection
// "Compute Aerodynamic Coefficients", where alpha = atan2(v_normal, v_chord).
struct BladeElementFlow {
  double v_chord_mps{};
  double v_normal_mps{};
};

// Inputs to the scalar pitch-moment model. The angular components are expressed
// in the wing-fixed aerodynamic basis from the .tex "Current Kinematic State"
// subsection and Appendix "Change of Basis Derivation"
// (sec:change_of_basis_derivation).
struct WingMomentInput {
  // One flow sample per blade station for the alpha, C_N, and aerodynamic
  // pitch-moment calculations.
  std::vector<BladeElementFlow> station_flows;
  // Passive pitch angle psi in M_x,hinge = -kappa_H psi.
  double pitch_angle_psi_rad{};
  // Wing-fixed angular velocity components omega_x, omega_y, omega_z.
  double omega_x_rad_s{};
  double omega_y_rad_s{};
  double omega_z_rad_s{};
  // Angular acceleration components required by "Compute added mass effect
  // moment". Only x and y enter the reduced M_x,am expression.
  double omega_dot_x_rad_s2{};
  double omega_dot_y_rad_s2{};
};

// Output terms mirror the moment decomposition in README/aeromechanical_model.tex.
// The simulator applies added mass as blade-element normal forces, so
// M_total = M_aero + M_rot + M_hinge for the scalar pitch-moment path.
struct WingMomentComponents {
  double angle_of_attack_alpha_rad{};
  double lift_N{};
  double drag_N{};
  double vertical_force_N{};
  double aerodynamic_Nm{};
  double rotational_damping_Nm{};
  double added_mass_Nm{};
  double hinge_Nm{};
  double total_Nm{};
  double applied_total_Nm{};
};

// README/aeromechanical_model.tex, "Compute Aerodynamic Coefficients":
// alpha = atan2(v_normal, v_chord).
inline double AngleOfAttackFromFlow(const BladeElementFlow& flow) {
  return std::atan2(flow.v_normal_mps, flow.v_chord_mps);
}

// Used for the sign convention in "Compute aerodynamic pitch moment about the
// hinge", where F_N = -sgn(alpha) F_N e_z.
inline double Sign(double value) {
  if (value > 0.0) return 1.0;
  if (value < 0.0) return -1.0;
  return 0.0;
}

// README/aeromechanical_model.tex, "Compute Aerodynamic Coefficients":
// C_L(alpha) = C_L,max sin(2 alpha).
inline double LiftCoefficient(double alpha_rad,
                              const AeromechanicalModelParameters& params) {
  return params.cl_max * std::sin(2.0 * alpha_rad);
}

// README/aeromechanical_model.tex, "Compute Aerodynamic Coefficients":
// C_D(alpha) = (C_D,max + C_D,0)/2 -
//              (C_D,max - C_D,0) cos(2 alpha)/2.
inline double DragCoefficient(double alpha_rad,
                              const AeromechanicalModelParameters& params) {
  return 0.5 * (params.cd_max + params.cd_0) -
         0.5 * (params.cd_max - params.cd_0) * std::cos(2.0 * alpha_rad);
}

// README/aeromechanical_model.tex, "Compute Aerodynamic Coefficients":
// C_N = C_L cos(alpha) + C_D sin(alpha).
inline double NormalForceCoefficient(
    double alpha_rad, const AeromechanicalModelParameters& params) {
  return LiftCoefficient(alpha_rad, params) * std::cos(alpha_rad) +
         DragCoefficient(alpha_rad, params) * std::sin(alpha_rad);
}

// README/aeromechanical_model.tex, "Compute passive hinge restoring moment":
// kappa_H = E_h t_h^3 w_h / (12 L_h).
inline double HingeTorsionalStiffness(
    const AeromechanicalModelParameters& params) {
  return params.kapton_youngs_modulus_pa *
         std::pow(params.hinge_thickness_m, 3) * params.hinge_width_m /
         (12.0 * params.hinge_length_m);
}

// Discrete blade-station width Delta r_i used by the blade-element sums in
// "Compute blade-element lift and drag" and "Compute aerodynamic pitch moment
// about the hinge". Interior stations use centered widths; endpoints get half
// of the neighboring interval.
template <std::size_t NumStations>
double StationWidth(const AeromechanicalWingConstants<NumStations>& constants,
                    int index) {
  const auto& stations = constants.blade_stations;
  if (stations.size() < 2) return 0.0;
  if (index == 0) return 0.5 * (stations[1].r_m - stations[0].r_m);
  if (index == static_cast<int>(stations.size()) - 1) {
    return 0.5 * (stations[index].r_m - stations[index - 1].r_m);
  }
  return 0.5 * (stations[index + 1].r_m - stations[index - 1].r_m);
}

// Computes the representative alpha used for logging. It follows the
// "Computing alpha" subsubsection, but averages station angles with weights
// proportional to V_i^2 c_i Delta r_i from eq:differential_lift_drag so the
// reported value reflects the strips doing most of the aerodynamic work.
template <std::size_t NumStations>
double CalcMeanAngleOfAttack(
    const AeromechanicalWingConstants<NumStations>& constants,
    const WingMomentInput& input) {
  double weighted_sin = 0.0;
  double weighted_cos = 0.0;
  const int count =
      std::min(static_cast<int>(constants.blade_stations.size()),
               static_cast<int>(input.station_flows.size()));

  for (int i = 0; i < count; ++i) {
    const auto& station = constants.blade_stations[i];
    const auto& flow = input.station_flows[i];
    const double dr = StationWidth(constants, i);
    if (!std::isfinite(dr) || dr <= 0.0 ||
        !std::isfinite(station.chord_m) || station.chord_m <= 0.0 ||
        !std::isfinite(flow.v_chord_mps) ||
        !std::isfinite(flow.v_normal_mps)) {
      continue;
    }

    const double speed_squared =
        flow.v_chord_mps * flow.v_chord_mps +
        flow.v_normal_mps * flow.v_normal_mps;
    if (!std::isfinite(speed_squared) || speed_squared <= 1.0e-16) continue;

    const double alpha_rad = AngleOfAttackFromFlow(flow);
    if (!std::isfinite(alpha_rad)) continue;

    const double weight = speed_squared * station.chord_m * dr;
    weighted_sin += weight * std::sin(alpha_rad);
    weighted_cos += weight * std::cos(alpha_rad);
  }

  const double alpha_rad = std::atan2(weighted_sin, weighted_cos);
  return std::isfinite(alpha_rad) ? alpha_rad : 0.0;
}

// README/aeromechanical_model.tex, "Compute aerodynamic pitch moment about the
// hinge": implements the discrete form
//   M_x,aero ~= -sum_i sgn(alpha_i) y_cp,i
//                    0.5 rho V_i^2 C_N(alpha_i) c_i Delta r_i.
// The center-of-pressure relation y_cp = y_r + y_LE - c d_cp_hat and
// d_cp_hat = 0.82 |alpha| / pi + 0.05 are implemented below. The lift/drag
// source term comes from eq:differential_lift_drag.
template <std::size_t NumStations>
double CalcAerodynamicPitchMoment(
    const AeromechanicalWingConstants<NumStations>& constants,
    const WingMomentInput& input,
    const AeromechanicalModelParameters& params) {
  double moment_Nm = 0.0;
  const int count =
      std::min(static_cast<int>(constants.blade_stations.size()),
               static_cast<int>(input.station_flows.size()));

  for (int i = 0; i < count; ++i) {
    const auto& station = constants.blade_stations[i];
    const auto& flow = input.station_flows[i];
    const double dr = StationWidth(constants, i);
    if (!std::isfinite(dr) || dr <= 0.0 ||
        !std::isfinite(station.chord_m) || station.chord_m <= 0.0 ||
        !std::isfinite(station.leading_edge_q_m) ||
        !std::isfinite(flow.v_chord_mps) ||
        !std::isfinite(flow.v_normal_mps)) {
      continue;
    }

    const double speed_squared =
        flow.v_chord_mps * flow.v_chord_mps +
        flow.v_normal_mps * flow.v_normal_mps;
    if (!std::isfinite(speed_squared) || speed_squared <= 1.0e-16) continue;

    const double alpha_rad = AngleOfAttackFromFlow(flow);
    const double alpha_abs_rad = std::abs(alpha_rad);
    // The paper's moment expression carries the aerodynamic direction in
    // sgn(alpha), so C_N and d_force_N are magnitudes here.
    const double c_n = NormalForceCoefficient(alpha_abs_rad, params);
    const double d_force_N =
        0.5 * params.air_density_kg_m3 * speed_squared * c_n *
        station.chord_m * dr;
    const double d_cp_hat =
        0.82 / kAeromechanicalPi * std::abs(alpha_rad) + 0.05;
    const double y_le_m =
        constants.y_r_hat_by_cbar * constants.mean_chord_cbar_m +
        station.leading_edge_q_m;
    const double y_cp_m = y_le_m - station.chord_m * d_cp_hat; // center of pressure 
    if (!std::isfinite(alpha_rad) || !std::isfinite(c_n) ||
        !std::isfinite(d_force_N) || !std::isfinite(y_cp_m)) {
      continue;
    }

    moment_Nm += -Sign(alpha_rad) * y_cp_m * d_force_N;
  }

  return std::isfinite(moment_Nm) ? moment_Nm : 0.0;
}

// README/aeromechanical_model.tex, "Compute aerodynamic damping moment":
// M_x,rd = -0.5 rho omega_x |omega_x| C_rd cbar^4 R Y_hat_rd.
template <std::size_t NumStations>
double CalcRotationalDampingMoment(
    const AeromechanicalWingConstants<NumStations>& constants,
    const WingMomentInput& input,
    const AeromechanicalModelParameters& params) {
  const double moment_Nm =
      -0.5 * params.air_density_kg_m3 * input.omega_x_rad_s *
      std::abs(input.omega_x_rad_s) * params.rotational_damping_coefficient *
      std::pow(constants.mean_chord_cbar_m, 4) * constants.span_R_m *
      constants.Y_rd_hat;
  return std::isfinite(moment_Nm) ? moment_Nm : 0.0;
}

// README/aeromechanical_model.tex, "Compute added mass effect moment":
// Implements the reduced M_x,am expression using the precomputed dimensionless
// integrals I_hat_xy,am and I_hat_xx,am from aeromechanical_wing_constants.h.
// The omega components are in the wing-fixed basis from Appendix
// sec:change_of_basis_derivation.
template <std::size_t NumStations>
double CalcAddedMassMoment(
    const AeromechanicalWingConstants<NumStations>& constants,
    const WingMomentInput& input,
    const AeromechanicalModelParameters& params) {
  const double coefficient_xy =
      kAeromechanicalPi / 4.0 * params.air_density_kg_m3 *
      std::pow(constants.mean_chord_cbar_m, 3) *
      constants.span_R_m * constants.span_R_m * constants.I_xy_am_hat;
  const double coefficient_xx =
      kAeromechanicalPi / 4.0 * params.air_density_kg_m3 *
      std::pow(constants.mean_chord_cbar_m, 4) *
      constants.span_R_m * constants.I_xx_am_hat;
  const double moment_Nm =
      -coefficient_xy *
          (input.omega_dot_y_rad_s2 -
           input.omega_x_rad_s * input.omega_z_rad_s) -
      coefficient_xx * input.omega_dot_x_rad_s2;
  return std::isfinite(moment_Nm) ? moment_Nm : 0.0;
}

// README/aeromechanical_model.tex, "Compute passive hinge restoring moment":
// M_x,hinge = -kappa_H psi.
inline double CalcHingeRestoringMoment(
    const WingMomentInput& input,
    const AeromechanicalModelParameters& params) {
  const double moment_Nm =
      -HingeTorsionalStiffness(params) * input.pitch_angle_psi_rad;
  return std::isfinite(moment_Nm) ? moment_Nm : 0.0;
}

// Aggregates the scalar pitch-moment terms from README/aeromechanical_model.tex.
// Added mass is still computed for diagnostics, but it is applied separately as
// blade-element normal forces in the simulator and is not summed into total_Nm.
// The full pitch-acceleration integration equation in that subsection is not
// solved here; the simulator applies this total moment to Drake instead.
template <std::size_t NumStations>
WingMomentComponents CalcAeromechanicalMoments(
    const AeromechanicalWingConstants<NumStations>& constants,
    const WingMomentInput& input,
    const AeromechanicalModelParameters& params =
        AeromechanicalModelParameters{}) {
  WingMomentComponents moments;
  moments.angle_of_attack_alpha_rad = CalcMeanAngleOfAttack(constants, input);
  moments.aerodynamic_Nm =
      CalcAerodynamicPitchMoment(constants, input, params);
  moments.rotational_damping_Nm =
      CalcRotationalDampingMoment(constants, input, params);
  moments.added_mass_Nm = CalcAddedMassMoment(constants, input, params);
  moments.hinge_Nm = CalcHingeRestoringMoment(input, params);
  moments.total_Nm = moments.aerodynamic_Nm + moments.rotational_damping_Nm +
                     moments.hinge_Nm;
  moments.applied_total_Nm = moments.total_Nm;
  if (std::isfinite(params.max_abs_applied_total_moment_Nm)) {
    moments.applied_total_Nm =
        std::clamp(moments.applied_total_Nm,
                   -params.max_abs_applied_total_moment_Nm,
                   params.max_abs_applied_total_moment_Nm);
  }
  if (!std::isfinite(moments.applied_total_Nm)) {
    moments.applied_total_Nm = 0.0;
  }
  return moments;
}

}  // namespace robobee