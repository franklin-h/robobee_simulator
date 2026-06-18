#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "aeromechanical_wing_constants.h"

namespace robobee {

constexpr double kAeromechanicalPi = 3.14159265358979323846;

struct AeromechanicalModelParameters {
  double air_density_kg_m3{1.225};
  double cl_max{1.8};
  double cd_max{3.4};
  double cd_0{0.4};
  double rotational_damping_coefficient{2.0};

  double kapton_youngs_modulus_pa{2.5e9};
  double hinge_thickness_m{0.025e-3};
  double hinge_width_m{2.7e-3};
  double hinge_length_m{0.10e-3};

  double max_abs_applied_total_moment_Nm{2.0e-05};
  double max_abs_total_aerodynamic_force_N{1.0e-02};
};

struct BladeElementFlow {
  double v_chord_mps{};
  double v_normal_mps{};
};

struct WingMomentInput {
  std::vector<BladeElementFlow> station_flows;
  double pitch_angle_psi_rad{};
  double omega_x_rad_s{};
  double omega_y_rad_s{};
  double omega_z_rad_s{};
  double omega_dot_x_rad_s2{};
  double omega_dot_y_rad_s2{};
};

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

inline double Sign(double value) {
  if (value > 0.0) return 1.0;
  if (value < 0.0) return -1.0;
  return 0.0;
}

inline double LiftCoefficient(double alpha_rad,
                              const AeromechanicalModelParameters& params) {
  return params.cl_max * std::sin(2.0 * alpha_rad);
}

inline double DragCoefficient(double alpha_rad,
                              const AeromechanicalModelParameters& params) {
  return 0.5 * (params.cd_max + params.cd_0) -
         0.5 * (params.cd_max - params.cd_0) * std::cos(2.0 * alpha_rad);
}

inline double NormalForceCoefficient(
    double alpha_rad, const AeromechanicalModelParameters& params) {
  return LiftCoefficient(alpha_rad, params) * std::cos(alpha_rad) +
         DragCoefficient(alpha_rad, params) * std::sin(alpha_rad);
}

inline double HingeTorsionalStiffness(
    const AeromechanicalModelParameters& params) {
  return params.kapton_youngs_modulus_pa *
         std::pow(params.hinge_thickness_m, 3) * params.hinge_width_m /
         (12.0 * params.hinge_length_m);
}

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

    const double alpha_rad =
        std::atan2(-flow.v_normal_mps, -flow.v_chord_mps);
    if (!std::isfinite(alpha_rad)) continue;

    const double weight = speed_squared * station.chord_m * dr;
    weighted_sin += weight * std::sin(alpha_rad);
    weighted_cos += weight * std::cos(alpha_rad);
  }

  const double alpha_rad = std::atan2(weighted_sin, weighted_cos);
  return std::isfinite(alpha_rad) ? alpha_rad : 0.0;
}

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

    const double alpha_rad =
        std::atan2(-flow.v_normal_mps, -flow.v_chord_mps);
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
    const double y_cp_m = y_le_m - station.chord_m * d_cp_hat;
    if (!std::isfinite(alpha_rad) || !std::isfinite(c_n) ||
        !std::isfinite(d_force_N) || !std::isfinite(y_cp_m)) {
      continue;
    }

    moment_Nm += -Sign(alpha_rad) * y_cp_m * d_force_N;
  }

  return std::isfinite(moment_Nm) ? moment_Nm : 0.0;
}

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

inline double CalcHingeRestoringMoment(
    const WingMomentInput& input,
    const AeromechanicalModelParameters& params) {
  const double moment_Nm =
      -HingeTorsionalStiffness(params) * input.pitch_angle_psi_rad;
  return std::isfinite(moment_Nm) ? moment_Nm : 0.0;
}

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
  moments.total_Nm = moments.aerodynamic_Nm*0 + moments.rotational_damping_Nm +
                     moments.added_mass_Nm + moments.hinge_Nm*0.25;
  // moments.total_Nm = moments.hinge_Nm*20 + moments.aerodynamic_Nm;
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
