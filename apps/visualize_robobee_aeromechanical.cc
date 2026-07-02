#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/shape_specification.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/coulomb_friction.h"
#include "drake/multibody/plant/externally_applied_spatial_force.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/joint_actuator.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/fixed_input_port_value.h"
#include "drake/systems/framework/leaf_system.h"

#include "aeromechanical_moments.h"
#ifdef ROBOBEE_SIMULINK_SERVER
#include "apps/robobee_simulink_tcp.h"
#endif

namespace {

// This executable builds a Drake diagram for the RoboBee assembly, drives the
// two slider joints with a sinusoidal stroke command, applies custom
// aeromechanical wing loads, publishes geometry to Meldis, and writes the
// per-wing moment breakdown to a CSV file.
constexpr char kPackageName[] = "robobee_assembly";
constexpr char kPackagePath[] = "models/robobee_assembly";
constexpr char kAssemblyUrl[] =
    "package://robobee_assembly/urdf/robobee_assembly.urdf";
constexpr double kPi = 3.14159265358979323846;

#ifdef ROBOBEE_FREE_FLIGHT
constexpr bool kFreeFlight = true;
#else
constexpr bool kFreeFlight = false;
#endif

// TODO: Add a GUI for simulation parameters and a placeholder section for
// RoboBee parameters.

constexpr double kMinStrokeGainMetersPerVolt = 1.3e-6;
constexpr double kMaxStrokeGainMetersPerVolt = 1.7e-6;

struct SimulationConfig {
  double drive_voltage_v{0.00025 / 1.5e-6};
  double stroke_gain_m_per_v{1.5e-6};
  int server_port{4242};
};

double CalcStrokeAmplitudeFromVoltage(double drive_voltage_v,
                                      double stroke_gain_m_per_v) {
  if (!std::isfinite(drive_voltage_v) || drive_voltage_v < 0.0) {
    throw std::runtime_error("Drive voltage must be finite and non-negative.");
  }
  if (!std::isfinite(stroke_gain_m_per_v) ||
      stroke_gain_m_per_v < kMinStrokeGainMetersPerVolt ||
      stroke_gain_m_per_v > kMaxStrokeGainMetersPerVolt) {
    throw std::runtime_error(
        "Stroke gain must be finite and in the range [1.3, 1.7] um/V.");
  }
  return stroke_gain_m_per_v * drive_voltage_v;
}

double CalcSignedStrokeAmplitudeFromDriveVoltage(double drive_voltage_v,
                                                 double bias_voltage_v,
                                                 double stroke_gain_m_per_v) {
  if (!std::isfinite(drive_voltage_v)) {
    throw std::runtime_error("Drive voltage command must be finite.");
  }
  if (!std::isfinite(bias_voltage_v) || bias_voltage_v < 0.0) {
    throw std::runtime_error(
        "Bias voltage command must be finite and non-negative.");
  }
  if (std::abs(drive_voltage_v) > 1.5 * bias_voltage_v) {
    throw std::runtime_error(
        "Drive voltage magnitude must not exceed bias voltage.");
  }
  if (!std::isfinite(stroke_gain_m_per_v) ||
      stroke_gain_m_per_v < kMinStrokeGainMetersPerVolt ||
      stroke_gain_m_per_v > kMaxStrokeGainMetersPerVolt) {
    throw std::runtime_error(
        "Stroke gain must be finite and in the range [1.3, 1.7] um/V.");
  }
  return stroke_gain_m_per_v * drive_voltage_v;
}

double ParseDoubleFlag(const std::string& arg, const std::string& flag_name) {
  const std::string prefix = "--" + flag_name + "=";
  if (arg.rfind(prefix, 0) != 0) {
    throw std::runtime_error("Internal parser error for flag " + flag_name);
  }
  try {
    std::size_t parsed_chars = 0;
    const double value = std::stod(arg.substr(prefix.size()), &parsed_chars);
    if (parsed_chars != arg.size() - prefix.size()) {
      throw std::runtime_error("");
    }
    return value;
  } catch (const std::exception&) {
    throw std::runtime_error("Could not parse --" + flag_name +
                             " as a double.");
  }
}

SimulationConfig ParseSimulationConfig(int argc, char** argv) {
  SimulationConfig config;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg.rfind("--drive_voltage=", 0) == 0) {
      config.drive_voltage_v = ParseDoubleFlag(arg, "drive_voltage");
    } else if (arg.rfind("--stroke_gain_um_per_v=", 0) == 0) {
      config.stroke_gain_m_per_v =
          1.0e-6 * ParseDoubleFlag(arg, "stroke_gain_um_per_v");
    } else if (arg.rfind("--server_port=", 0) == 0) {
      const double port = ParseDoubleFlag(arg, "server_port");
      if (port < 1 || port > 65535 || std::floor(port) != port) {
        throw std::runtime_error("--server_port must be an integer in [1, 65535].");
      }
      config.server_port = static_cast<int>(port);
    } else if (arg == "--help") {
      std::cout
          << "Usage: visualize_robobee_aeromechanical "
             "[--drive_voltage=V] [--stroke_gain_um_per_v=K] "
             "[--server_port=PORT]\n"
          << "  drive_voltage: initial actuator drive amplitude in volts\n"
          << "  stroke_gain_um_per_v: stroke gain K in um/V, range [1.3, 1.7]\n"
          << "  server_port: TCP port used by robobee_simulink_server\n";
      std::exit(0);
    } else {
      throw std::runtime_error("Unknown argument: " + arg);
    }
  }
  CalcStrokeAmplitudeFromVoltage(config.drive_voltage_v,
                                 config.stroke_gain_m_per_v);
  return config;
}

// Small Drake source system that produces the desired state vector expected by
// the plant's joint PD controllers: [q_slider_1, q_slider_2, v_slider_1,
// v_slider_2]. It accepts [right_drive_voltage, left_drive_voltage,
// bias_voltage]. The per-wing commands are signed drive amplitudes; the bias is
// a nonnegative common-mode safety voltage and does not directly create stroke.
class VoltageStrokeSource final : public drake::systems::LeafSystem<double> {
 public:
  VoltageStrokeSource(double stroke_gain_m_per_v, double drive_frequency_hz)
      : stroke_gain_m_per_v_(stroke_gain_m_per_v),
        drive_omega_(2.0 * kPi * drive_frequency_hz) {
    voltage_input_port_ =
        this->DeclareVectorInputPort("voltage_command", 3).get_index();
    this->DeclareVectorOutputPort("slider_desired_state", 4,
                                  &VoltageStrokeSource::CalcDesiredState);
  }

  const drake::systems::InputPort<double>& voltage_input_port() const {
    return this->get_input_port(voltage_input_port_);
  }

 private:
  void CalcDesiredState(const drake::systems::Context<double>& context,
                        drake::systems::BasicVector<double>* output) const {
    const double t = context.get_time();
    const Eigen::VectorXd& voltage_command =
        this->EvalVectorInput(context, voltage_input_port_)->get_value();
    const double right_voltage_v = voltage_command[0];
    const double left_voltage_v = voltage_command[1];
    const double bias_voltage_v = voltage_command[2];
    const double right_stroke_amplitude =
        CalcSignedStrokeAmplitudeFromDriveVoltage(
            right_voltage_v, bias_voltage_v, stroke_gain_m_per_v_);
    const double left_stroke_amplitude =
        CalcSignedStrokeAmplitudeFromDriveVoltage(
            left_voltage_v, bias_voltage_v, stroke_gain_m_per_v_);
    const double sin_drive = std::sin(drive_omega_ * t);
    const double cos_drive = std::cos(drive_omega_ * t);
    const double q_right = right_stroke_amplitude * sin_drive;
    const double q_left = left_stroke_amplitude * sin_drive;
    const double v_right = right_stroke_amplitude * drive_omega_ * cos_drive;
    const double v_left = left_stroke_amplitude * drive_omega_ * cos_drive;

    // Actuators are added below in this order: slider_1, then slider_2.
    Eigen::VectorBlock<Eigen::VectorXd> y = output->get_mutable_value();
    y << q_right, q_left, v_right, v_left;
  }

  drake::systems::InputPortIndex voltage_input_port_{};
  double stroke_gain_m_per_v_{};
  double drive_omega_{};
};

// Static information needed to interpret one wing body's kinematics in the
// aerodynamic coordinate system used by aeromechanical_moments.h.
struct WingAeroConfig {
  // Body name in the parsed URDF.
  std::string body_name;
  // Per-wing geometric constants and blade stations.
  const robobee::AeromechanicalWingConstants<400>* constants{};
  // Unit axes expressed in the Drake body frame B. The aerodynamic frame A uses
  // x = span, y = chord, z = normal.
  Eigen::Vector3d span_axis_B;
  Eigen::Vector3d chord_axis_B;
  Eigen::Vector3d normal_axis_B;
  // Sign convention for the scalar pitch moment applied about the span axis.
  double moment_sign{1.0};
  // Maps angular velocity/acceleration components from body frame B into the
  // aerodynamic component frame A.
  Eigen::Matrix3d X_AB{Eigen::Matrix3d::Identity()};
  // Initial world-frame chord direction. This is the zero-pitch reference used
  // to compute hinge restoring moment during the simulation.
  Eigen::Vector3d reference_chord_axis_W{Eigen::Vector3d::Zero()};
};

// History needed to estimate angular acceleration from consecutive angular
// velocity samples. Drake gives angular velocity directly, while the added-mass
// model also needs angular acceleration.
struct WingAngularHistory {
  bool initialized{false};
  double time_s{};
  Eigen::Vector3d omega_B{Eigen::Vector3d::Zero()};
  Eigen::Vector3d omega_dot_B{Eigen::Vector3d::Zero()};
};

// A blade element force is accumulated at a point fixed in the wing body frame.
// It is resolved in body axes until the final conversion required by Drake's
// ExternallyAppliedSpatialForce API.
struct BladeElementAppliedForce {
  Eigen::Vector3d p_BoBq_B{Eigen::Vector3d::Zero()};
  Eigen::Vector3d force_B{Eigen::Vector3d::Zero()};
};

// The blade-station data defines the span axis angle in the body xy-plane.
// Convert it to a normalized body-frame unit vector.
Eigen::Vector3d CalcAeroSpanAxisInBody(
    const robobee::AeromechanicalWingConstants<400>& constants) {
  const double c = std::cos(constants.span_axis_angle_rad);
  const double s = std::sin(constants.span_axis_angle_rad);
  return (c * Eigen::Vector3d::UnitX() + s * Eigen::Vector3d::UnitY())
      .normalized();
}

// The chord direction is perpendicular to span in the body xy-plane. The left
// and right wings can use opposite chord signs while sharing the same helper.
Eigen::Vector3d CalcAeroChordAxisInBody(
    const robobee::AeromechanicalWingConstants<400>& constants) {
  const double c = std::cos(constants.span_axis_angle_rad);
  const double s = std::sin(constants.span_axis_angle_rad);
  const double chord_sign = constants.chord_axis_sign >= 0.0 ? 1.0 : -1.0;
  return (chord_sign *
          (-s * Eigen::Vector3d::UnitX() + c * Eigen::Vector3d::UnitY()))
      .normalized();
}

// Build a component transform: multiplying X_AB by a vector expressed in B
// returns the scalar components along span, chord, and normal.
Eigen::Matrix3d CalcBodyToAeroComponentMatrix(
    const Eigen::Vector3d& span_axis_B, const Eigen::Vector3d& chord_axis_B,
    const Eigen::Vector3d& normal_axis_B) {
  Eigen::Matrix3d X_AB;
  X_AB.row(0) = span_axis_B.transpose();
  X_AB.row(1) = chord_axis_B.transpose();
  X_AB.row(2) = normal_axis_B.transpose();
  return X_AB;
}

// Guard the applied scalar pitch moment against invalid values and optional
// model limits. This prevents a single bad aerodynamic sample from destabilizing
// the multibody simulation.
double ClampMoment(double moment_Nm,
                   const robobee::AeromechanicalModelParameters& params) {
  if (!std::isfinite(moment_Nm)) return 0.0;
  if (!std::isfinite(params.max_abs_applied_total_moment_Nm)) {
    return moment_Nm;
  }
  const double limit = params.max_abs_applied_total_moment_Nm;
  if (limit < 0.0) return moment_Nm;
  return std::min(std::max(moment_Nm, -limit), limit);
}

// Custom Drake system that reads the MultibodyPlant state and outputs one
// ExternallyAppliedSpatialForce per wing. It performs a blade-element lift/drag
// calculation in body-fixed aerodynamic axes and adds the non-translational
// pitch moments from the aeromechanical model.
class WingAeromechanics final : public drake::systems::LeafSystem<double> {
 public:
  WingAeromechanics(const drake::multibody::MultibodyPlant<double>& plant,
                    double angular_acceleration_sample_period_s,
                    double angular_acceleration_filter_time_constant_s)
      : plant_(plant),
        plant_context_(plant.CreateDefaultContext()),
        angular_acceleration_sample_period_s_(
            angular_acceleration_sample_period_s),
        angular_acceleration_filter_time_constant_s_(
            angular_acceleration_filter_time_constant_s),
        wings_({WingAeroConfig{"wing_membrane",
                                &robobee::kLeftWingAeromechanicalConstants,
                                CalcAeroSpanAxisInBody(
                                    robobee::kLeftWingAeromechanicalConstants),
                                CalcAeroChordAxisInBody(
                                    robobee::kLeftWingAeromechanicalConstants),
                                Eigen::Vector3d::UnitZ(), 1.0},
                WingAeroConfig{"wing_membrane_1",
                                &robobee::kRightWingAeromechanicalConstants,
                                CalcAeroSpanAxisInBody(
                                    robobee::kRightWingAeromechanicalConstants),
                                CalcAeroChordAxisInBody(
                                    robobee::kRightWingAeromechanicalConstants),
                                Eigen::Vector3d::UnitZ(), 1.0}}) {
    // Cache the aerodynamic component transform and the initial chord direction
    // after the plant has loaded its default model configuration.
    for (WingAeroConfig& wing : wings_) {
      const auto& body = plant_.GetBodyByName(wing.body_name);
      const Eigen::Matrix3d R_WB0 =
          body.EvalPoseInWorld(*plant_context_).rotation().matrix();
      wing.X_AB = CalcBodyToAeroComponentMatrix(
          wing.span_axis_B, wing.chord_axis_B, wing.normal_axis_B);
      wing.reference_chord_axis_W = (R_WB0 * wing.chord_axis_B).normalized();
    }
    angular_histories_.resize(wings_.size());
    latest_moments_.resize(wings_.size());
    latest_pitch_angles_psi_rad_.resize(wings_.size(), 0.0);
    this->DeclareVectorInputPort("plant_state", plant.num_multibody_states());
    this->DeclareAbstractOutputPort("spatial_forces",
                                    &WingAeromechanics::CalcSpatialForces);
  }

  const std::vector<robobee::WingMomentComponents>& latest_moments() const {
    return latest_moments_;
  }

  const std::vector<double>& latest_pitch_angles_psi_rad() const {
    return latest_pitch_angles_psi_rad_;
  }

 private:
  static bool IsFinite(const Eigen::Vector3d& value) {
    return value.array().isFinite().all();
  }

  static double CalcPitchAngleAboutSpan(
      const WingAeroConfig& wing, const Eigen::Vector3d& span_axis_W,
      const Eigen::Vector3d& chord_axis_W) {
    // Remove any component along the current span axis, then compare the
    // projected initial and current chord directions with atan2 for a signed
    // rotation about span.
    const Eigen::Vector3d reference_chord_W =
        wing.reference_chord_axis_W -
        wing.reference_chord_axis_W.dot(span_axis_W) * span_axis_W;
    const Eigen::Vector3d current_chord_W =
        chord_axis_W - chord_axis_W.dot(span_axis_W) * span_axis_W;
    if (reference_chord_W.norm() <= 1.0e-12 ||
        current_chord_W.norm() <= 1.0e-12) {
      return 0.0;
    }
    const Eigen::Vector3d e_ref = reference_chord_W.normalized();
    const Eigen::Vector3d e_cur = current_chord_W.normalized();
    return std::atan2(span_axis_W.dot(e_ref.cross(e_cur)), e_ref.dot(e_cur));
  }

  Eigen::Vector3d EstimateAngularAccelerationInBody(
      int wing_index, double time_s, const Eigen::Vector3d& omega_B) const {
    WingAngularHistory& history = angular_histories_.at(wing_index);
    // First sample establishes the previous angular velocity. A zero
    // acceleration estimate is preferable to differencing against uninitialized
    // data.
    if (!history.initialized) {
      history.initialized = true;
      history.time_s = time_s;
      history.omega_B = omega_B;
      history.omega_dot_B.setZero();
      return history.omega_dot_B;
    }

    const double dt = time_s - history.time_s;
    if (dt <= 1.0e-12) {
      return history.omega_dot_B;
    }

    if (dt < angular_acceleration_sample_period_s_) {
      return history.omega_dot_B;
    }

    // Use a finite difference followed by a first-order low-pass filter. The
    // filter reduces numerical noise that would otherwise appear in the
    // added-mass moment.
    const Eigen::Vector3d raw_omega_dot_B = (omega_B - history.omega_B) / dt;
    if (!IsFinite(raw_omega_dot_B)) {
      return history.omega_dot_B;
    }

    const double filter_alpha =
        angular_acceleration_filter_time_constant_s_ > 0.0
            ? dt / (angular_acceleration_filter_time_constant_s_ + dt)
            : 1.0;
    history.omega_dot_B +=
        filter_alpha * (raw_omega_dot_B - history.omega_dot_B);
    history.omega_B = omega_B;
    history.time_s = time_s;
    return history.omega_dot_B;
  }

  void CalcSpatialForces(
      const drake::systems::Context<double>& context,
      std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>*
          output) const {
    // Mirror the live plant state into this system's private plant context so
    // body poses and velocities can be evaluated inside an output callback.
    const Eigen::VectorXd x =
        this->get_input_port(0).Eval(context);
    if (!x.array().isFinite().all()) {
      output->clear();
      return;
    }
    plant_.SetPositionsAndVelocities(plant_context_.get(), x);

    output->clear();
    output->reserve(wings_.size());

    for (int i = 0; i < static_cast<int>(wings_.size()); ++i) {
      AppendWingSpatialForces(wings_[i], i, context.get_time(), output);
    }
  }

  void AppendWingSpatialForces(
      const WingAeroConfig& wing, int wing_index, double time_s,
      std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>*
          output) const {
    const auto& constants = *wing.constants;
    const auto& body = plant_.GetBodyByName(wing.body_name);
    latest_moments_.at(wing_index) = {};
    latest_pitch_angles_psi_rad_.at(wing_index) = 0.0;
    const robobee::AeromechanicalModelParameters params;

    // Evaluate wing pose and spatial velocity in world frame W. Bad numeric
    // values are skipped instead of forwarded into the force input port.
    const drake::math::RigidTransformd& X_WB =
        body.EvalPoseInWorld(*plant_context_);
    const auto& V_WB = body.EvalSpatialVelocityInWorld(*plant_context_);
    if (!X_WB.GetAsMatrix4().array().isFinite().all() ||
        !IsFinite(V_WB.translational()) || !IsFinite(V_WB.rotational())) {
      return;
    }

    const Eigen::Matrix3d R_WB = X_WB.rotation().matrix();
    const Eigen::Matrix3d R_BW = R_WB.transpose();
    // Convert only the quantities Drake gives in world coordinates. The
    // aerodynamic force model below is resolved in body-fixed wing axes.
    const Eigen::Vector3d span_axis_W =
        (R_WB * wing.span_axis_B).normalized();
    const Eigen::Vector3d chord_axis_W =
        (R_WB * wing.chord_axis_B).normalized();
    const Eigen::Vector3d normal_axis_W =
        (R_WB * wing.normal_axis_B).normalized();
    if (!IsFinite(span_axis_W) || !IsFinite(chord_axis_W) ||
        !IsFinite(normal_axis_W)) {
      return;
    }
    const Eigen::Vector3d omega_h_W =
        V_WB.rotational() -
        V_WB.rotational().dot(span_axis_W) * span_axis_W;
    if (!IsFinite(omega_h_W)) {
      return;
    }

    const Eigen::Vector3d omega_B = R_BW * V_WB.rotational();
    const Eigen::Vector3d omega_dot_B =
        EstimateAngularAccelerationInBody(wing_index, time_s, omega_B);
    if (!IsFinite(omega_B) || !IsFinite(omega_dot_B)) {
      return;
    }
    const Eigen::Vector3d omega_A = wing.X_AB * omega_B;
    const Eigen::Vector3d omega_dot_A = wing.X_AB * omega_dot_B;

    const Eigen::Vector3d wind_W = Eigen::Vector3d::Zero();
    const auto& stations = constants.blade_stations;
    // moment_input.station_flows feeds the analytic moment model, while
    // blade_forces stores the actual distributed translational forces applied to
    // the Drake plant.
    robobee::WingMomentInput moment_input;
    moment_input.station_flows.reserve(stations.size());
    const double pitch_angle_psi_rad =
        CalcPitchAngleAboutSpan(wing, span_axis_W, chord_axis_W);
    moment_input.pitch_angle_psi_rad = pitch_angle_psi_rad;
    latest_pitch_angles_psi_rad_.at(wing_index) = pitch_angle_psi_rad;
    moment_input.omega_x_rad_s = omega_A.x();
    moment_input.omega_y_rad_s = omega_A.y();
    moment_input.omega_z_rad_s = omega_A.z();
    moment_input.omega_dot_x_rad_s2 = omega_dot_A.x();
    moment_input.omega_dot_y_rad_s2 = omega_dot_A.y();
    std::vector<BladeElementAppliedForce> blade_forces;
    blade_forces.reserve(stations.size());
    double total_blade_force_magnitude_N = 0.0;
    double total_added_mass_pitch_moment_Nm = 0.0;
    double total_lift_N = 0.0;
    double total_drag_N = 0.0;
    double weighted_alpha_sin = 0.0;
    double weighted_alpha_cos = 0.0;
    double total_alpha_weight = 0.0;

    // Integrate lift and drag over the blade stations. Each station represents
    // a spanwise strip with width dr and chord length station.chord_m.
    for (int i = 0; i < static_cast<int>(stations.size()); ++i) {
      const auto& station = stations[i];
      const double dr = robobee::StationWidth(constants, i);
      if (!std::isfinite(station.r_m) ||
          !std::isfinite(station.leading_edge_q_m) ||
          !std::isfinite(station.y_hinge_to_mid_chord_hat) ||
          !std::isfinite(station.chord_m) || station.chord_m <= 0.0 ||
          !std::isfinite(dr) || dr <= 0.0) {
        moment_input.station_flows.push_back({});
        continue;
      }

      const double x_r_m = constants.x_r_hat_by_R * constants.span_R_m;
      // Evaluate the velocity at the station mid-chord. The hinge-parallel
      // angular velocity component is removed so the local flow is driven by the
      // wing pitching/flapping motion relevant to the aerodynamic section.
      const Eigen::Vector3d p_BoP_B =
          (station.r_m + x_r_m) * wing.span_axis_B +
          station.y_hinge_to_mid_chord_hat * constants.mean_chord_cbar_m *
              wing.chord_axis_B;
      const Eigen::Vector3d p_BoP_W = R_WB * p_BoP_B;
      const Eigen::Vector3d v_P_W =
          V_WB.translational() + omega_h_W.cross(p_BoP_W);
      const Eigen::Vector3d v_rel_W = v_P_W - wind_W;
      if (!IsFinite(v_rel_W)) {
        moment_input.station_flows.push_back({});
        continue;
      }
      const Eigen::Vector3d v_rel_B = R_BW * v_rel_W;
      if (!IsFinite(v_rel_B)) {
        moment_input.station_flows.push_back({});
        continue;
      }
      const double v_chord_mps = v_rel_B.dot(wing.chord_axis_B);
      const double v_normal_mps = v_rel_B.dot(wing.normal_axis_B);
      moment_input.station_flows.push_back({v_chord_mps, v_normal_mps});

      // Apply added mass as a blade-element normal force at the local
      // mid-chord, rather than as a scalar pitch moment. This follows the
      // stripwise Z_0 expression documented in README/aeromechanical_model.tex.
      const double y_h_m =
          station.y_hinge_to_mid_chord_hat * constants.mean_chord_cbar_m;
      const double semi_chord_m = 0.5 * station.chord_m;
      const double lambda_z =
          robobee::kAeromechanicalPi * params.air_density_kg_m3 *
          semi_chord_m * semi_chord_m;
      const double lambda_zomega = -lambda_z * y_h_m;
      const double normal_acceleration_mps2 =
          -(station.r_m + x_r_m) *
          (omega_dot_A.y() - omega_A.x() * omega_A.z());
      const double added_mass_force_N =
          (-lambda_z * normal_acceleration_mps2 -
           lambda_zomega * omega_dot_A.x()) *
          dr;
      if (std::isfinite(added_mass_force_N)) {
        BladeElementAppliedForce added_force;
        added_force.p_BoBq_B = p_BoP_B;
        added_force.force_B = added_mass_force_N * wing.normal_axis_B;
        blade_forces.push_back(added_force);
        total_blade_force_magnitude_N += added_force.force_B.norm();
        total_added_mass_pitch_moment_Nm +=
            p_BoP_B.cross(added_force.force_B).dot(wing.span_axis_B);
      }

      const double speed_squared =
          v_chord_mps * v_chord_mps + v_normal_mps * v_normal_mps;
      if (!std::isfinite(speed_squared) || speed_squared <= 1.0e-16) {
        continue;
      }

      const double speed_mps = std::sqrt(speed_squared);
      const double alpha_rad = std::atan2(v_normal_mps, v_chord_mps);
      if (!std::isfinite(alpha_rad)) continue;
      // Circular averaging keeps the reported mean angle well behaved near
      // +/-pi, and weighting by dynamic-pressure area emphasizes active strips.
      const double alpha_weight = speed_squared * station.chord_m * dr;
      weighted_alpha_sin += alpha_weight * std::sin(alpha_rad);
      weighted_alpha_cos += alpha_weight * std::cos(alpha_rad);
      total_alpha_weight += alpha_weight;

      const double d_cp_hat =
          0.82 / robobee::kAeromechanicalPi * std::abs(alpha_rad) + 0.05;
      const double y_le_m =
          constants.y_r_hat_by_cbar * constants.mean_chord_cbar_m +
          station.leading_edge_q_m;
      const double y_cp_m = y_le_m - station.chord_m * d_cp_hat;
      if (!std::isfinite(y_cp_m)) continue;

      // Resolve the sectional force into a drag component opposite relative
      // motion and a lift component perpendicular to drag within the local
      // chord-normal plane.

      // Basis for Eq. 48/49 in aeromechanical_model.tex
      const double dynamic_pressure_times_area =
          0.5 * params.air_density_kg_m3 * speed_squared * station.chord_m *
          dr;
      const Eigen::Vector3d v_rel_in_aero_plane_B =
          v_chord_mps * wing.chord_axis_B + v_normal_mps * wing.normal_axis_B;
      const Eigen::Vector3d drag_axis_B =
          -v_rel_in_aero_plane_B / speed_mps;
      const Eigen::Vector3d lift_axis_B =
          wing.span_axis_B.cross(drag_axis_B).normalized();
      const double d_lift_N =
          dynamic_pressure_times_area *
          robobee::LiftCoefficient(alpha_rad, params);
      const double d_drag_N =
          dynamic_pressure_times_area *
          robobee::DragCoefficient(alpha_rad, params);
      const Eigen::Vector3d blade_force_B =
          d_lift_N * lift_axis_B + d_drag_N * drag_axis_B;
      if (!IsFinite(blade_force_B)) continue;

      BladeElementAppliedForce blade_force;
      // Apply the strip force at the estimated center of pressure rather than
      // the mid-chord point used for local velocity sampling.
      blade_force.p_BoBq_B =
          (station.r_m + x_r_m) * wing.span_axis_B +
          y_cp_m * wing.chord_axis_B;
      blade_force.force_B = blade_force_B;
      blade_forces.push_back(blade_force);
      total_blade_force_magnitude_N += blade_force_B.norm();
      total_lift_N += d_lift_N;
      total_drag_N += d_drag_N;
    }

    // Optionally scale all translational strip forces together so the net
    // applied force respects the model's configured cap.
    double force_scale = 1.0;
    if (std::isfinite(params.max_abs_total_aerodynamic_force_N) &&
        params.max_abs_total_aerodynamic_force_N >= 0.0 &&
        std::isfinite(total_blade_force_magnitude_N) &&
        total_blade_force_magnitude_N >
            params.max_abs_total_aerodynamic_force_N) {
      force_scale = params.max_abs_total_aerodynamic_force_N /
                    total_blade_force_magnitude_N;
    }
    Eigen::Vector3d total_blade_force_W = Eigen::Vector3d::Zero();
    Eigen::Vector3d total_blade_moment_W = Eigen::Vector3d::Zero();
    // Collapse the distributed strip forces into one spatial force at the body
    // origin. Convert the body-fixed strip forces to world frame only at this
    // Drake interface boundary.
    for (const BladeElementAppliedForce& blade_force : blade_forces) {
      const Eigen::Vector3d scaled_force_W =
          R_WB * (force_scale * blade_force.force_B);
      total_blade_force_W += scaled_force_W;
      total_blade_moment_W +=
          (R_WB * blade_force.p_BoBq_B).cross(scaled_force_W);
    }

    robobee::WingMomentComponents moments =
        robobee::CalcAeromechanicalMoments(constants, moment_input, params);

    if (total_alpha_weight > 0.0) {
      moments.angle_of_attack_alpha_rad =
          std::atan2(weighted_alpha_sin, weighted_alpha_cos);
    }
    moments.lift_N = force_scale * total_lift_N;
    moments.drag_N = force_scale * total_drag_N;
    moments.vertical_force_N = total_blade_force_W.z();

    // Log the added-mass pitch moment that is actually produced by the
    // distributed added-mass strip forces.
    moments.added_mass_Nm = force_scale * total_added_mass_pitch_moment_Nm;

    // The translational aerodynamic and added-mass pitch moments are already
    // represented by distributed strip forces above. Apply only the remaining
    // scalar pitch moments, such as rotational damping and hinge restoring,
    // about the span axis.
    const double non_translational_pitch_moment_Nm = ClampMoment(
        moments.rotational_damping_Nm + moments.hinge_Nm, params);

    // Log the span-axis moment that is actually sent to Drake.
    moments.applied_total_Nm =
        total_blade_moment_W.dot(span_axis_W) +
        wing.moment_sign * non_translational_pitch_moment_Nm;
    if (!std::isfinite(moments.applied_total_Nm)) {
      moments.applied_total_Nm = 0.0;
    }
    latest_moments_.at(wing_index) = moments;

    drake::multibody::ExternallyAppliedSpatialForce<double> force;
    force.body_index = body.index();
    // p_BoBq_B = 0 means the spatial force is applied at the wing body origin;
    // F_Bq_W carries both the moment about that point and the net force, both
    // expressed in world coordinates as Drake requires.
    force.p_BoBq_B = Eigen::Vector3d::Zero();
    force.F_Bq_W = drake::multibody::SpatialForce<double>(
        total_blade_moment_W +
            wing.moment_sign * non_translational_pitch_moment_Nm * span_axis_W,
        total_blade_force_W);
    output->push_back(force);
  }

  const drake::multibody::MultibodyPlant<double>& plant_;
  mutable std::unique_ptr<drake::systems::Context<double>> plant_context_;
  double angular_acceleration_sample_period_s_{};
  double angular_acceleration_filter_time_constant_s_{};
  std::vector<WingAeroConfig> wings_;
  mutable std::vector<WingAngularHistory> angular_histories_;
  mutable std::vector<robobee::WingMomentComponents> latest_moments_;
  mutable std::vector<double> latest_pitch_angles_psi_rad_;
};

void RegisterRoboBeePackage(drake::multibody::Parser* parser) {
  parser->package_map().Add(kPackageName, kPackagePath);
}

// Load a temporary copy of the model in its default pose so we can recover the
// fixed transform from a loop-closure helper body to the real link it should
// coincide with. The production plant then uses that transform as a weld
// constraint.
drake::math::RigidTransformd CalcDefaultPoseOfLoopFrameInRealLinkFrame(
    const std::string& loop_body_name, const std::string& real_body_name) {
  drake::multibody::MultibodyPlant<double> plant(0.0);
  drake::multibody::Parser parser(&plant);
  RegisterRoboBeePackage(&parser);
  parser.AddModelsFromUrl(kAssemblyUrl);
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("root"));
  plant.Finalize();

  std::unique_ptr<drake::systems::Context<double>> context =
      plant.CreateDefaultContext();
  return plant.CalcRelativeTransform(*context,
                                     plant.GetFrameByName(real_body_name),
                                     plant.GetFrameByName(loop_body_name));
}

// Recover a loop-frame origin and its local y-axis, expressed in body frame B,
// from the model's default configuration. Two points along this axis are later
// constrained with ball constraints to emulate an aligned hinge axis.
void CalcDefaultLoopFramePointAndAxis(
    const std::string& loop_body_name, const std::string& body_name,
    Eigen::Vector3d* p_BL, Eigen::Vector3d* axis_B) {
  drake::multibody::MultibodyPlant<double> plant(0.0);
  drake::multibody::Parser parser(&plant);
  RegisterRoboBeePackage(&parser);
  parser.AddModelsFromUrl(kAssemblyUrl);
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("root"));
  plant.Finalize();

  std::unique_ptr<drake::systems::Context<double>> context =
      plant.CreateDefaultContext();
  const drake::math::RigidTransformd X_BL =
      plant.CalcRelativeTransform(*context, plant.GetFrameByName(body_name),
                                  plant.GetFrameByName(loop_body_name));

  *p_BL = X_BL.translation();
  *axis_B = X_BL.rotation() * Eigen::Vector3d(0.0, 1.0, 0.0);
}

// Weld the URDF's loop-closure placeholder body to the corresponding physical
// link. This keeps the duplicated loop body coincident with the modeled link.
void AddLoopClosureWeldConstraint(
    drake::multibody::MultibodyPlant<double>* plant,
    const std::string& loop_body_name, const std::string& real_body_name) {
  const drake::math::RigidTransformd X_RealLoop =
      CalcDefaultPoseOfLoopFrameInRealLinkFrame(loop_body_name, real_body_name);

  plant->AddWeldConstraint(
      plant->GetBodyByName(loop_body_name), drake::math::RigidTransformd(),
      plant->GetBodyByName(real_body_name), X_RealLoop);
}

// Add two ball constraints between the hinge body and the real transmission
// link. A single ball constraint matches one point; the second offset point also
// aligns the hinge axis and removes the relative twist that would otherwise
// remain.
void AddLoopClosureBallConstraints(
    drake::multibody::MultibodyPlant<double>* plant,
    const std::string& loop_body_name, const std::string& hinge_body_name,
    const std::string& real_link_body_name) {
  Eigen::Vector3d p_HL;
  Eigen::Vector3d axis_H;
  CalcDefaultLoopFramePointAndAxis(loop_body_name, hinge_body_name, &p_HL,
                                   &axis_H);

  Eigen::Vector3d p_RL;
  Eigen::Vector3d axis_R;
  CalcDefaultLoopFramePointAndAxis(loop_body_name, real_link_body_name, &p_RL,
                                   &axis_R);

  constexpr double kAxisPointOffset = 1.0e-4;
  plant->AddBallConstraint(plant->GetBodyByName(hinge_body_name), p_HL,
                           plant->GetBodyByName(real_link_body_name), p_RL);
  plant->AddBallConstraint(plant->GetBodyByName(hinge_body_name),
                           p_HL + kAxisPointOffset * axis_H.normalized(),
                           plant->GetBodyByName(real_link_body_name),
                           p_RL + kAxisPointOffset * axis_R.normalized());
}

// Draw a small RGB frame on a body in Meldis so the wing body axes can be
// checked visually while tuning aerodynamic axes and signs.
void AddBodyFrameTriadVisual(drake::multibody::MultibodyPlant<double>* plant,
                             const std::string& body_name,
                             const std::string& name_prefix) {
  constexpr double kAxisLength = 2.0e-3;
  constexpr double kAxisRadius = 3.5e-5;
  constexpr double kOriginRadius = 6.0e-5;

  const auto& body = plant->GetBodyByName(body_name);
  const auto add_axis =
      [&](const std::string& suffix,
          const drake::math::RotationMatrixd& R_BC,
          const Eigen::Vector3d& p_BC,
          const Eigen::Vector4d& color) {
        plant->RegisterVisualGeometry(
            body, drake::math::RigidTransformd(R_BC, p_BC),
            drake::geometry::Cylinder(kAxisRadius, kAxisLength),
            name_prefix + "_" + suffix, color);
      };

  plant->RegisterVisualGeometry(
      body, drake::math::RigidTransformd::Identity(),
      drake::geometry::Sphere(kOriginRadius), name_prefix + "_origin",
      Eigen::Vector4d(1.0, 1.0, 1.0, 1.0));
  add_axis("x", drake::math::RotationMatrixd::MakeYRotation(0.5 * kPi),
           0.5 * kAxisLength * Eigen::Vector3d::UnitX(),
           Eigen::Vector4d(1.0, 0.0, 0.0, 1.0));
  add_axis("y", drake::math::RotationMatrixd::MakeXRotation(-0.5 * kPi),
           0.5 * kAxisLength * Eigen::Vector3d::UnitY(),
           Eigen::Vector4d(0.0, 0.8, 0.0, 1.0));
  add_axis("z", drake::math::RotationMatrixd::Identity(),
           0.5 * kAxisLength * Eigen::Vector3d::UnitZ(),
           Eigen::Vector4d(0.0, 0.2, 1.0, 1.0));
}

// Optional free-flight floor. It is compiled in only when ROBOBEE_FREE_FLIGHT
// is defined, otherwise the root is welded to world in main().
[[maybe_unused]] void AddWorldFloor(
    drake::multibody::MultibodyPlant<double>* plant) {
  constexpr double kFloorThickness = 1.0e-3;
  constexpr double kFloorSize = 0.20;
  const auto& world = plant->world_body();

  plant->RegisterCollisionGeometry(
      world, drake::math::RigidTransformd::Identity(),
      drake::geometry::HalfSpace(), "floor_collision",
      drake::multibody::CoulombFriction<double>(1.0, 1.0));
  plant->RegisterVisualGeometry(
      world,
      drake::math::RigidTransformd(
          Eigen::Vector3d(0.0, 0.0, -0.5 * kFloorThickness)),
      drake::geometry::Box(kFloorSize, kFloorSize, kFloorThickness),
      "floor_visual", Eigen::Vector4d(0.55, 0.58, 0.62, 0.35));
}

// CSV utilities for the moment log. The order matches the left/right entries in
// WingAeromechanics::wings_.
void WriteMomentCsvHeader(std::ostream* output) {
  *output
      << "time_s,"
      << "left_psi_rad,"
      << "left_alpha_rad,left_lift_N,left_drag_N,left_force_z_N,"
         "left_aero_Nm,left_rot_Nm,"
         "left_added_Nm,left_hinge_Nm,left_total_Nm,left_applied_Nm,"
      << "right_psi_rad,"
      << "right_alpha_rad,right_lift_N,right_drag_N,right_force_z_N,"
         "right_aero_Nm,"
         "right_rot_Nm,right_added_Nm,right_hinge_Nm,right_total_Nm,"
         "right_applied_Nm\n";
}

void WriteMomentCsvRow(
    std::ostream* output, double time_s,
    const std::vector<robobee::WingMomentComponents>& moments,
    const std::vector<double>& pitch_angles_psi_rad) {
  const robobee::WingMomentComponents zero;
  const robobee::WingMomentComponents& left =
      moments.size() > 0 ? moments[0] : zero;
  const robobee::WingMomentComponents& right =
      moments.size() > 1 ? moments[1] : zero;
  const double left_psi_rad =
      pitch_angles_psi_rad.size() > 0 ? pitch_angles_psi_rad[0] : 0.0;
  const double right_psi_rad =
      pitch_angles_psi_rad.size() > 1 ? pitch_angles_psi_rad[1] : 0.0;

  *output << std::setprecision(17) << time_s << ','
          << left_psi_rad << ','
          << left.angle_of_attack_alpha_rad << ','
          << left.lift_N << ',' << left.drag_N << ','
          << left.vertical_force_N << ','
          << left.aerodynamic_Nm << ',' << left.rotational_damping_Nm << ','
          << left.added_mass_Nm << ',' << left.hinge_Nm << ','
          << left.total_Nm << ',' << left.applied_total_Nm << ','
          << right_psi_rad << ','
          << right.angle_of_attack_alpha_rad << ','
          << right.lift_N << ',' << right.drag_N << ','
          << right.vertical_force_N << ','
          << right.aerodynamic_Nm << ',' << right.rotational_damping_Nm << ','
          << right.added_mass_Nm << ',' << right.hinge_Nm << ','
          << right.total_Nm << ',' << right.applied_total_Nm << '\n';
}

double CalcSliderDesiredPosition(double time_s, double stroke_amplitude,
                                 double drive_frequency_hz) {
  return stroke_amplitude * std::sin(2.0 * kPi * drive_frequency_hz * time_s);
}

void WriteSliderCsvHeader(std::ostream* output) {
  *output << "time_s,right_actual_m,left_actual_m,"
             "right_desired_m,left_desired_m,"
             "right_error_m,left_error_m,"
             "drive_voltage_v,stroke_gain_m_per_v\n";
}

void WriteSliderCsvRow(std::ostream* output, double time_s,
                       double right_actual_m, double left_actual_m,
                       double desired_m, double drive_voltage_v,
                       double stroke_gain_m_per_v) {
  *output << std::setprecision(17) << time_s << ','
          << right_actual_m << ',' << left_actual_m << ','
          << desired_m << ',' << desired_m << ','
          << right_actual_m - desired_m << ','
          << left_actual_m - desired_m << ','
          << drive_voltage_v << ',' << stroke_gain_m_per_v << '\n';
}

struct RobotPose {
  Eigen::Vector3d p_WR{Eigen::Vector3d::Zero()};
  Eigen::Vector3d roll_pitch_yaw_rad{Eigen::Vector3d::Zero()};
};

RobotPose CalcRobotPose(
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::systems::Context<double>& plant_context) {
  const drake::math::RigidTransformd X_WR =
      plant.GetBodyByName("root").EvalPoseInWorld(plant_context);
  RobotPose pose;
  pose.p_WR = X_WR.translation();
  pose.roll_pitch_yaw_rad = X_WR.rotation().ToRollPitchYaw().vector();
  return pose;
}

void WritePoseCsvHeader(std::ostream* output) {
  *output << "time_s,x_m,y_m,z_m,roll_rad,pitch_rad,yaw_rad\n";
}

void WritePoseCsvRow(std::ostream* output, double time_s,
                     const RobotPose& pose) {
  *output << std::setprecision(17) << time_s << ','
          << pose.p_WR.x() << ',' << pose.p_WR.y() << ',' << pose.p_WR.z()
          << ',' << pose.roll_pitch_yaw_rad.x() << ','
          << pose.roll_pitch_yaw_rad.y() << ','
          << pose.roll_pitch_yaw_rad.z() << '\n';
}

#ifdef ROBOBEE_SIMULINK_SERVER

namespace tcp = ::robobee::simulink;

class RobobeeSimulationServer final {
 public:
  explicit RobobeeSimulationServer(const SimulationConfig& config)
      : config_stroke_gain_m_per_v_(config.stroke_gain_m_per_v) {
    constexpr double kDriveFrequencyHz = 180;
    constexpr double kPlantStepsPerDriveCycle = 100.0;
    constexpr double kAngularAccelerationSamplesPerDriveCycle =
        kPlantStepsPerDriveCycle;
    constexpr double kAngularAccelerationFilterCycles = 0.05;
    constexpr double kPlantTimeStep =
        1.0 / (kDriveFrequencyHz * kPlantStepsPerDriveCycle);
    constexpr double kAngularAccelerationSamplePeriod =
        1.0 / (kDriveFrequencyHz * kAngularAccelerationSamplesPerDriveCycle);
    constexpr double kAngularAccelerationFilterTimeConstant =
        kAngularAccelerationFilterCycles / kDriveFrequencyHz;
    constexpr double kVisualizerPublishPeriod = 1.0 / 120.0*5;

    auto [plant, scene_graph] =
        drake::multibody::AddMultibodyPlantSceneGraph(&builder_,
                                                      kPlantTimeStep);
    plant_ = &plant;
    scene_graph_ = &scene_graph;
    plant_->set_discrete_contact_approximation(
        drake::multibody::DiscreteContactApproximation::kSap);

    drake::multibody::Parser parser(plant_);
    RegisterRoboBeePackage(&parser);
    const std::vector<drake::multibody::ModelInstanceIndex> model_instances =
        parser.AddModelsFromUrl(kAssemblyUrl);
    if (model_instances.size() != 1) {
      throw std::runtime_error("Expected exactly one RoboBee model instance.");
    }
    const drake::multibody::ModelInstanceIndex model_instance =
        model_instances.front();

    if constexpr (kFreeFlight) {
      AddWorldFloor(plant_);
    } else {
      plant_->WeldFrames(plant_->world_frame(), plant_->GetFrameByName("root"));
    }

    AddLoopClosureWeldConstraint(plant_,
                                 "transmission_link_2__1__loop_closure",
                                 "transmission_link_2");
    AddLoopClosureWeldConstraint(plant_,
                                 "transmission_right_link_2__1__loop_closure",
                                 "transmission_right_link_2");
    AddLoopClosureBallConstraints(plant_,
                                  "transmission_link_2__1__loop_closure",
                                  "transmission_hinge",
                                  "transmission_link_2");
    AddLoopClosureBallConstraints(plant_,
                                  "transmission_right_link_2__1__loop_closure",
                                  "transmission_right_link_hinge",
                                  "transmission_right_link_2");

    constexpr double kSliderEffortLimitN = 100000.0;
    constexpr double kSliderKp = 1000.0;
    constexpr double kSliderKd = 5.0e-3;

    const auto& right_slider_actuator = plant_->AddJointActuator(
        "right_slider_drive", plant_->GetJointByName("slider_1"),
        kSliderEffortLimitN);
    plant_->get_mutable_joint_actuator(right_slider_actuator.index())
        .set_controller_gains({kSliderKp, kSliderKd});

    const auto& left_slider_actuator = plant_->AddJointActuator(
        "left_slider_drive", plant_->GetJointByName("slider_2"),
        kSliderEffortLimitN);
    plant_->get_mutable_joint_actuator(left_slider_actuator.index())
        .set_controller_gains({kSliderKp, kSliderKd});

    plant_->Finalize();

    slider_source_ = builder_.AddSystem<VoltageStrokeSource>(
        config.stroke_gain_m_per_v, kDriveFrequencyHz);
    builder_.Connect(slider_source_->get_output_port(),
                     plant_->get_desired_state_input_port(model_instance));

    auto* wing_aero = builder_.AddSystem<WingAeromechanics>(
        *plant_, kAngularAccelerationSamplePeriod,
        kAngularAccelerationFilterTimeConstant);
    builder_.Connect(plant_->get_state_output_port(),
                     wing_aero->get_input_port(0));
    builder_.Connect(wing_aero->get_output_port(0),
                     plant_->get_applied_spatial_force_input_port());

    drake::geometry::DrakeVisualizerParams visualizer_params;
    visualizer_params.publish_period = kVisualizerPublishPeriod;
    drake::geometry::DrakeVisualizerd::AddToBuilder(
        &builder_, scene_graph, &lcm_, visualizer_params);

    diagram_ = builder_.Build();
    simulator_ = std::make_unique<drake::systems::Simulator<double>>(*diagram_);
    simulator_->set_target_realtime_rate(0.0);
    InitializeVoltageCommand(config.drive_voltage_v, config.drive_voltage_v,
                             config.drive_voltage_v);
    if constexpr (kFreeFlight) {
      auto& root_context = simulator_->get_mutable_context();
      auto& plant_context = plant_->GetMyMutableContextFromRoot(&root_context);
      plant_->SetFreeBodyPose(
          &plant_context, plant_->GetBodyByName("root"),
          drake::math::RigidTransformd(Eigen::Vector3d(0.0, 0.0, 1.0e-2)));
    }
    simulator_->Initialize();
    drake::geometry::DrakeVisualizerd::DispatchLoadMessage(*scene_graph_,
                                                           &lcm_);
  }

  tcp::StepResponse Step(const tcp::StepRequest& request) {
    if (!std::isfinite(request.dt_s) || request.dt_s <= 0.0) {
      throw std::runtime_error("Step request dt_s must be finite and positive.");
    }
    const double right_actuator_voltage_v =
        kVoltageAmplifierGain * request.right_voltage_v;
    const double left_actuator_voltage_v =
        kVoltageAmplifierGain * request.left_voltage_v;
    const double bias_actuator_voltage_v =
        kVoltageAmplifierGain * request.bias_voltage_v;
    LogReceivedVoltageCommand(request, right_actuator_voltage_v,
                              left_actuator_voltage_v,
                              bias_actuator_voltage_v);
    FixVoltageCommand(right_actuator_voltage_v, left_actuator_voltage_v,
                      bias_actuator_voltage_v);
    time_s_ += request.dt_s;
    simulator_->AdvanceTo(time_s_);

    const auto& root_context = simulator_->get_context();
    const auto& plant_context = plant_->GetMyContextFromRoot(root_context);
    const RobotPose pose = CalcRobotPose(*plant_, plant_context);

    tcp::StepResponse response;
    response.time_s = time_s_;
    response.x_m = pose.p_WR.x();
    response.y_m = pose.p_WR.y();
    response.z_m = pose.p_WR.z();
    response.roll_rad = pose.roll_pitch_yaw_rad.x();
    response.pitch_rad = pose.roll_pitch_yaw_rad.y();
    response.yaw_rad = pose.roll_pitch_yaw_rad.z();
    return response;
  }

 private:
  static constexpr double kVoltageAmplifierGain = 100.0;
  static constexpr int kVoltageLogPeriodSteps = 1000;

  void LogReceivedVoltageCommand(const tcp::StepRequest& request,
                                 double right_actuator_voltage_v,
                                 double left_actuator_voltage_v,
                                 double bias_actuator_voltage_v) {
    ++voltage_log_count_;
    const double max_abs_drive_preamp_v =
        std::max(std::abs(request.left_voltage_v),
                 std::abs(request.right_voltage_v));
    const double max_abs_drive_actuator_v =
        std::max(std::abs(left_actuator_voltage_v),
                 std::abs(right_actuator_voltage_v));
    const bool violates_bias_limit =
        !std::isfinite(bias_actuator_voltage_v) ||
        bias_actuator_voltage_v < 0.0 ||
        max_abs_drive_actuator_v > bias_actuator_voltage_v;
    if (voltage_log_count_ <= 10 ||
        voltage_log_count_ % kVoltageLogPeriodSteps == 0 ||
        violates_bias_limit) {
      std::cout << std::setprecision(6)
                << "Voltage command step " << voltage_log_count_
                << " at t=" << time_s_
                << " s: preamp |drive|max=" << max_abs_drive_preamp_v
                << " V, preamp bias=" << request.bias_voltage_v
                << " V; actuator |drive|max=" << max_abs_drive_actuator_v
                << " V, actuator bias=" << bias_actuator_voltage_v << " V";
      if (violates_bias_limit) {
        std::cout << "  [violates |drive| <= bias]";
      }
      std::cout << "\n";
    }
  }

  void FixVoltageCommand(double right_voltage_v, double left_voltage_v,
                         double bias_voltage_v) {
    CalcSignedStrokeAmplitudeFromDriveVoltage(
        right_voltage_v, bias_voltage_v, config_stroke_gain_m_per_v_);
    CalcSignedStrokeAmplitudeFromDriveVoltage(
        left_voltage_v, bias_voltage_v, config_stroke_gain_m_per_v_);
    voltage_input_value_->GetMutableVectorData<double>()->get_mutable_value()
        << right_voltage_v, left_voltage_v, bias_voltage_v;
  }

  void InitializeVoltageCommand(double right_voltage_v, double left_voltage_v,
                                double bias_voltage_v) {
    CalcSignedStrokeAmplitudeFromDriveVoltage(
        right_voltage_v, bias_voltage_v, config_stroke_gain_m_per_v_);
    CalcSignedStrokeAmplitudeFromDriveVoltage(
        left_voltage_v, bias_voltage_v, config_stroke_gain_m_per_v_);
    auto& root_context = simulator_->get_mutable_context();
    auto& source_context =
        slider_source_->GetMyMutableContextFromRoot(&root_context);
    Eigen::Vector3d voltage_command;
    voltage_command << right_voltage_v, left_voltage_v, bias_voltage_v;
    voltage_input_value_ = &slider_source_->voltage_input_port().FixValue(
        &source_context, voltage_command);
  }

  drake::systems::DiagramBuilder<double> builder_;
  drake::multibody::MultibodyPlant<double>* plant_{};
  drake::geometry::SceneGraph<double>* scene_graph_{};
  drake::lcm::DrakeLcm lcm_;
  VoltageStrokeSource* slider_source_{};
  drake::systems::FixedInputPortValue* voltage_input_value_{};
  std::unique_ptr<drake::systems::Diagram<double>> diagram_;
  std::unique_ptr<drake::systems::Simulator<double>> simulator_;
  double config_stroke_gain_m_per_v_{1.5e-6};
  double time_s_{0.0};
  int voltage_log_count_{0};
};

#endif  // ROBOBEE_SIMULINK_SERVER

}  // namespace

#ifndef ROBOBEE_SIMULINK_SERVER
int main(int argc, char** argv) {
  const SimulationConfig config = ParseSimulationConfig(argc, argv);
  const double kSliderAmplitude = CalcStrokeAmplitudeFromVoltage(
      config.drive_voltage_v, config.stroke_gain_m_per_v);

  drake::systems::DiagramBuilder<double> builder;

  // Simulation timing is expressed in samples per wingbeat cycle so the plant,
  // visualizer, moment logger, and finite-difference acceleration estimator stay
  // synchronized as drive frequency changes.
  constexpr double kDriveFrequencyHz = 180;
  constexpr double kPlantStepsPerDriveCycle = 100.0;
  constexpr double kVisualizerSamplesPerDriveCycle = kPlantStepsPerDriveCycle*2;
  constexpr double kMomentLogSamplesPerDriveCycle = kPlantStepsPerDriveCycle;
  constexpr double kAngularAccelerationSamplesPerDriveCycle =
      kPlantStepsPerDriveCycle;
  constexpr double kAngularAccelerationFilterCycles = 0.05;
  constexpr double kPlantTimeStep =
      1.0 / (kDriveFrequencyHz * kPlantStepsPerDriveCycle);
  constexpr double kVisualizerPublishPeriod =
      1.0 / (kDriveFrequencyHz * kVisualizerSamplesPerDriveCycle);
  constexpr double kMomentLogPeriod =
      1.0 / (kDriveFrequencyHz * kMomentLogSamplesPerDriveCycle);
  constexpr double kAngularAccelerationSamplePeriod =
      1.0 / (kDriveFrequencyHz * kAngularAccelerationSamplesPerDriveCycle);
  constexpr double kAngularAccelerationFilterTimeConstant =
      kAngularAccelerationFilterCycles / kDriveFrequencyHz;
  constexpr double kCommandPeriod = kPlantTimeStep;
  constexpr double kMomentLogFlushPeriod = 5.0e-2;
  constexpr double kSliderLogPeriod = kMomentLogPeriod;
  constexpr double kSliderLogFlushPeriod = kMomentLogFlushPeriod;
  constexpr double kPoseLogPeriod = kMomentLogPeriod;
  constexpr double kPoseLogFlushPeriod = kMomentLogFlushPeriod;
  constexpr double kTargetRealtimeRate = 0.02;
  constexpr char kMomentLogPath[] = "/tmp/aeromechanical_moments.csv";
  constexpr char kSliderLogPath[] = "/tmp/slider_positions.csv";
  constexpr char kPoseLogPath[] = "/tmp/robobee_pose.csv";

  // A discrete MultibodyPlant is used because the assembly has constraints and
  // the wing loads are applied at a high update rate.
  auto [plant, scene_graph] =
      drake::multibody::AddMultibodyPlantSceneGraph(&builder, kPlantTimeStep);
  plant.set_discrete_contact_approximation(
      drake::multibody::DiscreteContactApproximation::kSap);

  drake::multibody::Parser parser(&plant);
  RegisterRoboBeePackage(&parser);
  const std::vector<drake::multibody::ModelInstanceIndex> model_instances =
      parser.AddModelsFromUrl(kAssemblyUrl);
  if (model_instances.size() != 1) {
    throw std::runtime_error("Expected exactly one RoboBee model instance.");
  }
  const drake::multibody::ModelInstanceIndex model_instance =
      model_instances.front();

  // Free-flight builds leave the root body floating above a floor; the default
  // visualization build welds the root to world so the linkage motion is easier
  // to inspect.
  if constexpr (kFreeFlight) {
    AddWorldFloor(&plant);
  } else {
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("root"));
  }

  // Close the transmission loops with Drake constraints. The weld constraints
  // keep loop-closure placeholder bodies attached to their real links, and the
  // paired ball constraints align the hinge points and axes.
  AddLoopClosureWeldConstraint(&plant,
                               "transmission_link_2__1__loop_closure",
                               "transmission_link_2");
  AddLoopClosureWeldConstraint(&plant,
                               "transmission_right_link_2__1__loop_closure",
                               "transmission_right_link_2");
  AddLoopClosureBallConstraints(&plant,
                                "transmission_link_2__1__loop_closure",
                                "transmission_hinge",
                                "transmission_link_2");
  AddLoopClosureBallConstraints(&plant,
                                "transmission_right_link_2__1__loop_closure",
                                "transmission_right_link_hinge",
                                "transmission_right_link_2");

  // constexpr double kSliderEffortLimitN = 1.0e-1;
  // The high effort limit makes the slider source behave like a prescribed
  // stroke while still using Drake's finite-gain actuator path.
  constexpr double kSliderEffortLimitN = 100000.0;
  constexpr double kSliderKp = 1000.0;
  constexpr double kSliderKd = 5.0e-3;

  // Add PD-controlled actuators to the two slider joints. The desired state
  // input is provided after Finalize by VoltageStrokeSource.
  const auto& right_slider_actuator = plant.AddJointActuator(
      "right_slider_drive", plant.GetJointByName("slider_1"),
      kSliderEffortLimitN);
  plant.get_mutable_joint_actuator(right_slider_actuator.index())
      .set_controller_gains({kSliderKp, kSliderKd});

  const auto& left_slider_actuator = plant.AddJointActuator(
      "left_slider_drive", plant.GetJointByName("slider_2"),
      kSliderEffortLimitN);
  plant.get_mutable_joint_actuator(left_slider_actuator.index())
      .set_controller_gains({kSliderKp, kSliderKd});

  AddBodyFrameTriadVisual(&plant, "wing_membrane", "left_wing_membrane_frame");
  AddBodyFrameTriadVisual(&plant, "wing_membrane_1",
                          "right_wing_membrane_frame");

  // After Finalize the plant topology is fixed. The diagram can still connect
  // systems to the plant's existing input ports.
  plant.Finalize();

  // Diagram wiring:
  //   slider source -> plant desired joint state
  //   plant state   -> wing aeromechanics
  //   wing forces   -> plant applied spatial force input
  auto* slider_source = builder.AddSystem<VoltageStrokeSource>(
      config.stroke_gain_m_per_v, kDriveFrequencyHz);
  builder.Connect(slider_source->get_output_port(),
                  plant.get_desired_state_input_port(model_instance));

  auto* wing_aero = builder.AddSystem<WingAeromechanics>(
      plant, kAngularAccelerationSamplePeriod,
      kAngularAccelerationFilterTimeConstant);
  builder.Connect(plant.get_state_output_port(),
                  wing_aero->get_input_port(0));
  builder.Connect(wing_aero->get_output_port(0),
                  plant.get_applied_spatial_force_input_port());

  // Publish geometry on LCM so Meldis can display the assembly and the wing
  // frame triads while the simulation is running.
  drake::lcm::DrakeLcm lcm;
  drake::geometry::DrakeVisualizerParams visualizer_params;
  visualizer_params.publish_period = kVisualizerPublishPeriod;
  drake::geometry::DrakeVisualizerd::AddToBuilder(
      &builder, scene_graph, &lcm, visualizer_params);

  auto diagram = builder.Build();
  drake::systems::Simulator<double> simulator(*diagram);
  simulator.set_target_realtime_rate(kTargetRealtimeRate);
  {
    auto& root_context = simulator.get_mutable_context();
    auto& source_context =
        slider_source->GetMyMutableContextFromRoot(&root_context);
    Eigen::Vector3d voltage_command;
    voltage_command << config.drive_voltage_v, config.drive_voltage_v,
        config.drive_voltage_v;
    slider_source->voltage_input_port().FixValue(&source_context,
                                                 voltage_command);
  }
  if constexpr (kFreeFlight) {
    // Start slightly above the z=0 floor to avoid initial penetration.
    auto& root_context = simulator.get_mutable_context();
    auto& plant_context = plant.GetMyMutableContextFromRoot(&root_context);
    plant.SetFreeBodyPose(
        &plant_context, plant.GetBodyByName("root"),
        drake::math::RigidTransformd(Eigen::Vector3d(0.0, 0.0, 1.0e-2)));
  }
  const auto& right_slider =
      plant.GetJointByName<drake::multibody::PrismaticJoint>("slider_1");
  const auto& left_slider =
      plant.GetJointByName<drake::multibody::PrismaticJoint>("slider_2");
  simulator.Initialize();

  std::cout << "RoboBee constrained linkage model is being simulated and "
               "published on LCM for Meldis.\n"
            << "Start Meldis before or after this process:\n"
            << "  bazel run @drake//tools:meldis -- --open-window\n\n"
            << "Aeromechanical moments are being logged to:\n"
            << "  " << kMomentLogPath << "\n\n"
            << "Slider actual/desired positions are being logged to:\n"
            << "  " << kSliderLogPath << "\n"
            << "Plot them with:\n"
            << "  python3 tools/plot_slider_positions.py\n\n"
            << "Robot pose for Simulink-style feedback is being logged to:\n"
            << "  " << kPoseLogPath << "\n\n"
            << "Voltage command:\n"
            << "  drive voltage amplitude: " << config.drive_voltage_v
            << " V\n"
            << "  stroke gain K: " << config.stroke_gain_m_per_v * 1.0e6
            << " um/V\n"
            << "  stroke amplitude: " << kSliderAmplitude << " m\n\n"
            << "Timing:\n"
            << "  drive frequency: " << kDriveFrequencyHz << " Hz\n"
            << "  plant timestep: " << kPlantTimeStep << " s ("
            << kPlantStepsPerDriveCycle << " steps/cycle)\n"
            << "  visualizer period: " << kVisualizerPublishPeriod << " s ("
            << kVisualizerSamplesPerDriveCycle << " samples/cycle)\n"
            << "  moment log period: " << kMomentLogPeriod << " s ("
            << kMomentLogSamplesPerDriveCycle << " samples/cycle)\n"
            << "  angular acceleration sample period: "
            << kAngularAccelerationSamplePeriod << " s ("
            << kAngularAccelerationSamplesPerDriveCycle << " samples/cycle)\n"
            << "  angular acceleration filter time constant: "
            << kAngularAccelerationFilterTimeConstant << " s ("
            << kAngularAccelerationFilterCycles << " cycles)\n"
            << "  target realtime rate: " << kTargetRealtimeRate << "\n"
            << "  free flight: " << (kFreeFlight ? "yes" : "no") << "\n\n"
            << "The slider joints are driven by finite-gain joint actuators "
               "tracking a sinusoidal desired position and velocity; "
               "the transmission loops are closed with MultibodyPlant weld "
               "constraints, not per-frame IK. "
            << (kFreeFlight
                    ? "The root body is floating and a z=0 floor is enabled.\n"
                    : "The root body is welded to world.\n")
            << "Press Ctrl-C here to stop publishing.\n";

  // Send the scene description once before time stepping so Meldis can load
  // geometry even if it starts listening after this process begins.
  drake::geometry::DrakeVisualizerd::DispatchLoadMessage(scene_graph, &lcm);

  std::ofstream moment_log(kMomentLogPath);
  if (!moment_log) {
    throw std::runtime_error("Could not open aeromechanical moment CSV log.");
  }
  WriteMomentCsvHeader(&moment_log);

  std::ofstream slider_log(kSliderLogPath);
  if (!slider_log) {
    throw std::runtime_error("Could not open slider position CSV log.");
  }
  WriteSliderCsvHeader(&slider_log);

  std::ofstream pose_log(kPoseLogPath);
  if (!pose_log) {
    throw std::runtime_error("Could not open RoboBee pose CSV log.");
  }
  WritePoseCsvHeader(&pose_log);

  double next_time = 0.0;
  double next_moment_log_time = 0.0;
  double next_moment_log_flush_time = kMomentLogFlushPeriod;
  double next_slider_log_time = 0.0;
  double next_slider_log_flush_time = kSliderLogFlushPeriod;
  double next_pose_log_time = 0.0;
  double next_pose_log_flush_time = kPoseLogFlushPeriod;
  // Manual stepping gives this app explicit control over log cadence and flush
  // cadence. The simulator is otherwise advanced one plant step at a time.
  while (true) {
    next_time += kCommandPeriod;
    simulator.AdvanceTo(next_time);
    if (next_time + 0.5 * kCommandPeriod >= next_moment_log_time) {
      WriteMomentCsvRow(&moment_log, next_time, wing_aero->latest_moments(),
                        wing_aero->latest_pitch_angles_psi_rad());
      next_moment_log_time += kMomentLogPeriod;
    }
    if (next_time + 0.5 * kCommandPeriod >= next_slider_log_time) {
      const auto& root_context = simulator.get_context();
      const auto& plant_context = plant.GetMyContextFromRoot(root_context);
      const double desired_slider_position = CalcSliderDesiredPosition(
          next_time, kSliderAmplitude, kDriveFrequencyHz);
      WriteSliderCsvRow(&slider_log, next_time,
                        right_slider.get_translation(plant_context),
                        left_slider.get_translation(plant_context),
                        desired_slider_position, config.drive_voltage_v,
                        config.stroke_gain_m_per_v);
      next_slider_log_time += kSliderLogPeriod;
    }
    if (next_time + 0.5 * kCommandPeriod >= next_pose_log_time) {
      const auto& root_context = simulator.get_context();
      const auto& plant_context = plant.GetMyContextFromRoot(root_context);
      WritePoseCsvRow(&pose_log, next_time,
                      CalcRobotPose(plant, plant_context));
      next_pose_log_time += kPoseLogPeriod;
    }
    if (next_time + 0.5 * kCommandPeriod >= next_moment_log_flush_time) {
      moment_log.flush();
      next_moment_log_flush_time += kMomentLogFlushPeriod;
    }
    if (next_time + 0.5 * kCommandPeriod >= next_slider_log_flush_time) {
      slider_log.flush();
      next_slider_log_flush_time += kSliderLogFlushPeriod;
    }
    if (next_time + 0.5 * kCommandPeriod >= next_pose_log_flush_time) {
      pose_log.flush();
      next_pose_log_flush_time += kPoseLogFlushPeriod;
    }
  }

  return 0;
}
#else
int main(int argc, char** argv) {
  const SimulationConfig config = ParseSimulationConfig(argc, argv);
  tcp::FileDescriptor listen_fd = tcp::ListenOnLocalhost(config.server_port);

  std::cout << "RoboBee Simulink server is listening on 127.0.0.1:"
            << config.server_port << ".\n"
            << "Protocol: request = [dt_s, left_voltage_v, right_voltage_v, "
               "bias_voltage_v], "
               "response = [time_s, x_m, y_m, z_m, roll_rad, pitch_rad, "
               "yaw_rad], all binary double values.\n"
            << "Voltage request fields are pre-amplifier commands; the server "
               "applies a 100x voltage amplifier gain before driving Drake.\n"
            << "Waiting for Simulink client...\n";

  while (true) {
    tcp::FileDescriptor client_fd = tcp::AcceptClient(listen_fd.get());
    std::cout << "Simulink client connected; initializing Drake simulation.\n";
    try {
      RobobeeSimulationServer simulator(config);
      while (true) {
        tcp::StepRequest request;
        if (!tcp::RecvExact(client_fd.get(), &request, sizeof(request))) {
          break;
        }
        const tcp::StepResponse response = simulator.Step(request);
        tcp::SendExact(client_fd.get(), &response, sizeof(response));
      }
      std::cout << "Simulink client disconnected; waiting for a new client.\n";
    } catch (const std::exception& e) {
      std::cerr << "Client session ended with error: " << e.what() << "\n"
                << "Waiting for a new client.\n";
    }
  }
}
#endif
