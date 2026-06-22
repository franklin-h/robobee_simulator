#include <algorithm>
#include <cmath>
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
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/externally_applied_spatial_force.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/joint_actuator.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"

#include "aeromechanical_moments.h"

namespace {

constexpr char kPackageName[] = "robobee_assembly";
constexpr char kPackagePath[] = "models/robobee_assembly";
constexpr char kAssemblyUrl[] =
    "package://robobee_assembly/urdf/robobee_assembly.urdf";
constexpr double kPi = 3.14159265358979323846;

/* 
TODO: Have a GUI interface with simulation parameters for a section. And also a section (placeholder for now) which will have robobee parameters. 
*/

class SliderStrokeSource final : public drake::systems::LeafSystem<double> {
 public:
  SliderStrokeSource(double stroke_amplitude, double drive_frequency_hz)
      : stroke_amplitude_(stroke_amplitude),
        drive_omega_(2.0 * kPi * drive_frequency_hz) {
    this->DeclareVectorOutputPort("slider_desired_state", 4,
                                  &SliderStrokeSource::CalcDesiredState);
  }

 private:
  void CalcDesiredState(const drake::systems::Context<double>& context,
                        drake::systems::BasicVector<double>* output) const {
    const double t = context.get_time();
    const double q = stroke_amplitude_ * std::sin(drive_omega_ * t);
    const double v =
        stroke_amplitude_ * drive_omega_ * std::cos(drive_omega_ * t);

    // Actuators are added below in this order: slider_1, then slider_2.
    Eigen::VectorBlock<Eigen::VectorXd> y = output->get_mutable_value();
    y << q, q, v, v;
  }

  double stroke_amplitude_{};
  double drive_omega_{};
};

struct WingAeroConfig {
  std::string body_name;
  const robobee::AeromechanicalWingConstants<400>* constants{};
  Eigen::Vector3d span_axis_B;
  Eigen::Vector3d chord_axis_B;
  Eigen::Vector3d normal_axis_B;
  double moment_sign{1.0};
  Eigen::Matrix3d X_AB{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d reference_chord_axis_W{Eigen::Vector3d::Zero()};
};

struct WingAngularHistory {
  bool initialized{false};
  double time_s{};
  Eigen::Vector3d omega_B{Eigen::Vector3d::Zero()};
  Eigen::Vector3d omega_dot_B{Eigen::Vector3d::Zero()};
};

struct BladeElementAppliedForce {
  Eigen::Vector3d p_BoBq_B{Eigen::Vector3d::Zero()};
  Eigen::Vector3d force_W{Eigen::Vector3d::Zero()};
};

Eigen::Vector3d CalcAeroSpanAxisInBody(
    const robobee::AeromechanicalWingConstants<400>& constants) {
  const double c = std::cos(constants.span_axis_angle_rad);
  const double s = std::sin(constants.span_axis_angle_rad);
  return (c * Eigen::Vector3d::UnitX() + s * Eigen::Vector3d::UnitY())
      .normalized();
}

Eigen::Vector3d CalcAeroChordAxisInBody(
    const robobee::AeromechanicalWingConstants<400>& constants) {
  const double c = std::cos(constants.span_axis_angle_rad);
  const double s = std::sin(constants.span_axis_angle_rad);
  const double chord_sign = constants.chord_axis_sign >= 0.0 ? 1.0 : -1.0;
  return (chord_sign *
          (-s * Eigen::Vector3d::UnitX() + c * Eigen::Vector3d::UnitY()))
      .normalized();
}

Eigen::Matrix3d CalcBodyToAeroComponentMatrix(
    const Eigen::Vector3d& span_axis_B, const Eigen::Vector3d& chord_axis_B,
    const Eigen::Vector3d& normal_axis_B) {
  Eigen::Matrix3d X_AB;
  X_AB.row(0) = span_axis_B.transpose();
  X_AB.row(1) = chord_axis_B.transpose();
  X_AB.row(2) = normal_axis_B.transpose();
  return X_AB;
}

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
    this->DeclareVectorInputPort("plant_state", plant.num_multibody_states());
    this->DeclareAbstractOutputPort("spatial_forces",
                                    &WingAeromechanics::CalcSpatialForces);
  }

  const std::vector<robobee::WingMomentComponents>& latest_moments() const {
    return latest_moments_;
  }

 private:
  static bool IsFinite(const Eigen::Vector3d& value) {
    return value.array().isFinite().all();
  }

  static double CalcPitchAngleAboutSpan(
      const WingAeroConfig& wing, const Eigen::Vector3d& span_axis_W,
      const Eigen::Vector3d& chord_axis_W) {
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
    const robobee::AeromechanicalModelParameters params;

    const drake::math::RigidTransformd& X_WB =
        body.EvalPoseInWorld(*plant_context_);
    const auto& V_WB = body.EvalSpatialVelocityInWorld(*plant_context_);
    if (!X_WB.GetAsMatrix4().array().isFinite().all() ||
        !IsFinite(V_WB.translational()) || !IsFinite(V_WB.rotational())) {
      return;
    }

    const Eigen::Matrix3d R_WB = X_WB.rotation().matrix();
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

    const Eigen::Vector3d wind_W = Eigen::Vector3d::Zero();
    const auto& stations = constants.blade_stations;
    robobee::WingMomentInput moment_input;
    moment_input.station_flows.reserve(stations.size());
    std::vector<BladeElementAppliedForce> blade_forces;
    blade_forces.reserve(stations.size());
    double total_blade_force_magnitude_N = 0.0;
    double total_lift_N = 0.0;
    double total_drag_N = 0.0;
    double weighted_alpha_sin = 0.0;
    double weighted_alpha_cos = 0.0;
    double total_alpha_weight = 0.0;

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
      const double v_chord_mps = v_rel_W.dot(chord_axis_W);
      const double v_normal_mps = v_rel_W.dot(normal_axis_W);
      moment_input.station_flows.push_back({v_chord_mps, v_normal_mps});

      const double speed_squared =
          v_chord_mps * v_chord_mps + v_normal_mps * v_normal_mps;
      if (!std::isfinite(speed_squared) || speed_squared <= 1.0e-16) {
        continue;
      }

      const double speed_mps = std::sqrt(speed_squared);
      const double alpha_rad = std::atan2(v_normal_mps, v_chord_mps);
      if (!std::isfinite(alpha_rad)) continue;
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

      const double dynamic_pressure_times_area =
          0.5 * params.air_density_kg_m3 * speed_squared * station.chord_m *
          dr;
      const Eigen::Vector3d v_rel_in_aero_plane_W =
          v_chord_mps * chord_axis_W + v_normal_mps * normal_axis_W;
      const Eigen::Vector3d drag_axis_W =
          -v_rel_in_aero_plane_W / speed_mps;
      const Eigen::Vector3d lift_axis_W =
          span_axis_W.cross(drag_axis_W).normalized();
      const double d_lift_N =
          dynamic_pressure_times_area *
          robobee::LiftCoefficient(alpha_rad, params);
      const double d_drag_N =
          dynamic_pressure_times_area *
          robobee::DragCoefficient(alpha_rad, params);
      const Eigen::Vector3d blade_force_W =
          d_lift_N * lift_axis_W + d_drag_N * drag_axis_W;
      if (!IsFinite(blade_force_W)) continue;

      BladeElementAppliedForce blade_force;
      blade_force.p_BoBq_B =
          (station.r_m + x_r_m) * wing.span_axis_B +
          y_cp_m * wing.chord_axis_B;
      blade_force.force_W = blade_force_W;
      blade_forces.push_back(blade_force);
      total_blade_force_magnitude_N += blade_force_W.norm();
      total_lift_N += d_lift_N;
      total_drag_N += d_drag_N;
    }

    double force_scale = 1.0;
    if (std::isfinite(params.max_abs_total_aerodynamic_force_N) &&
        params.max_abs_total_aerodynamic_force_N >= 0.0 &&
        std::isfinite(total_blade_force_magnitude_N) &&
        total_blade_force_magnitude_N >
            params.max_abs_total_aerodynamic_force_N) {
      force_scale = params.max_abs_total_aerodynamic_force_N /
                    total_blade_force_magnitude_N;
    }
    Eigen::Vector3d total_aero_force_W = Eigen::Vector3d::Zero();
    Eigen::Vector3d total_aero_moment_W = Eigen::Vector3d::Zero();
    for (const BladeElementAppliedForce& blade_force : blade_forces) {
      const Eigen::Vector3d scaled_force_W = force_scale * blade_force.force_W;
      total_aero_force_W += scaled_force_W;
      total_aero_moment_W +=
          (R_WB * blade_force.p_BoBq_B).cross(scaled_force_W);
    }

    const Eigen::Vector3d omega_B = R_WB.transpose() * V_WB.rotational();
    const Eigen::Vector3d omega_dot_B =
        EstimateAngularAccelerationInBody(wing_index, time_s, omega_B);
    if (!IsFinite(omega_B) || !IsFinite(omega_dot_B)) {
      return;
    }

    moment_input.pitch_angle_psi_rad =
        CalcPitchAngleAboutSpan(wing, span_axis_W, chord_axis_W);
    const Eigen::Vector3d omega_A = wing.X_AB * omega_B;
    const Eigen::Vector3d omega_dot_A = wing.X_AB * omega_dot_B;
    moment_input.omega_x_rad_s = omega_A.x();
    moment_input.omega_y_rad_s = omega_A.y();
    moment_input.omega_z_rad_s = omega_A.z();
    moment_input.omega_dot_x_rad_s2 = omega_dot_A.x();
    moment_input.omega_dot_y_rad_s2 = omega_dot_A.y();

    robobee::WingMomentComponents moments =
        robobee::CalcAeromechanicalMoments(constants, moment_input, params);
    if (total_alpha_weight > 0.0) {
      moments.angle_of_attack_alpha_rad =
          std::atan2(weighted_alpha_sin, weighted_alpha_cos);
    }
    moments.lift_N = force_scale * total_lift_N;
    moments.drag_N = force_scale * total_drag_N;
    moments.vertical_force_N = total_aero_force_W.z();
    const double non_translational_pitch_moment_Nm =
        ClampMoment(moments.total_Nm - moments.aerodynamic_Nm, params);
    moments.applied_total_Nm =
        force_scale * moments.aerodynamic_Nm +
        non_translational_pitch_moment_Nm;
    if (!std::isfinite(moments.applied_total_Nm)) {
      moments.applied_total_Nm = 0.0;
    }
    latest_moments_.at(wing_index) = moments;

    drake::multibody::ExternallyAppliedSpatialForce<double> force;
    force.body_index = body.index();
    force.p_BoBq_B = Eigen::Vector3d::Zero();
    force.F_Bq_W = drake::multibody::SpatialForce<double>(
        total_aero_moment_W +
            wing.moment_sign * non_translational_pitch_moment_Nm * span_axis_W,
        total_aero_force_W);
    output->push_back(force);
  }

  const drake::multibody::MultibodyPlant<double>& plant_;
  mutable std::unique_ptr<drake::systems::Context<double>> plant_context_;
  double angular_acceleration_sample_period_s_{};
  double angular_acceleration_filter_time_constant_s_{};
  std::vector<WingAeroConfig> wings_;
  mutable std::vector<WingAngularHistory> angular_histories_;
  mutable std::vector<robobee::WingMomentComponents> latest_moments_;
};

void RegisterRoboBeePackage(drake::multibody::Parser* parser) {
  parser->package_map().Add(kPackageName, kPackagePath);
}

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

void AddLoopClosureWeldConstraint(
    drake::multibody::MultibodyPlant<double>* plant,
    const std::string& loop_body_name, const std::string& real_body_name) {
  const drake::math::RigidTransformd X_RealLoop =
      CalcDefaultPoseOfLoopFrameInRealLinkFrame(loop_body_name, real_body_name);

  plant->AddWeldConstraint(
      plant->GetBodyByName(loop_body_name), drake::math::RigidTransformd(),
      plant->GetBodyByName(real_body_name), X_RealLoop);
}

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

void WriteMomentCsvHeader(std::ostream* output) {
  *output
      << "time_s,"
      << "left_alpha_rad,left_lift_N,left_drag_N,left_force_z_N,"
         "left_aero_Nm,left_rot_Nm,"
         "left_added_Nm,left_hinge_Nm,left_total_Nm,left_applied_Nm,"
      << "right_alpha_rad,right_lift_N,right_drag_N,right_force_z_N,"
         "right_aero_Nm,"
         "right_rot_Nm,right_added_Nm,right_hinge_Nm,right_total_Nm,"
         "right_applied_Nm\n";
}

void WriteMomentCsvRow(
    std::ostream* output, double time_s,
    const std::vector<robobee::WingMomentComponents>& moments) {
  const robobee::WingMomentComponents zero;
  const robobee::WingMomentComponents& left =
      moments.size() > 0 ? moments[0] : zero;
  const robobee::WingMomentComponents& right =
      moments.size() > 1 ? moments[1] : zero;

  *output << std::setprecision(17) << time_s << ','
          << left.angle_of_attack_alpha_rad << ','
          << left.lift_N << ',' << left.drag_N << ','
          << left.vertical_force_N << ','
          << left.aerodynamic_Nm << ',' << left.rotational_damping_Nm << ','
          << left.added_mass_Nm << ',' << left.hinge_Nm << ','
          << left.total_Nm << ',' << left.applied_total_Nm << ','
          << right.angle_of_attack_alpha_rad << ','
          << right.lift_N << ',' << right.drag_N << ','
          << right.vertical_force_N << ','
          << right.aerodynamic_Nm << ',' << right.rotational_damping_Nm << ','
          << right.added_mass_Nm << ',' << right.hinge_Nm << ','
          << right.total_Nm << ',' << right.applied_total_Nm << '\n';
}

}  // namespace

int main() {
  drake::systems::DiagramBuilder<double> builder;

  constexpr double kSliderAmplitude = 0.00030;  // 0.6 mm peak-to-peak.
  constexpr double kDriveFrequencyHz = 180;
  constexpr double kPlantStepsPerDriveCycle = 60.0;
  constexpr double kVisualizerSamplesPerDriveCycle = kPlantStepsPerDriveCycle;
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
  constexpr double kTargetRealtimeRate = 0.02;
  constexpr char kMomentLogPath[] = "/tmp/aeromechanical_moments.csv";

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

  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("root"));

  // more rigid constraints to ensure 
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
  constexpr double kSliderEffortLimitN = 1.0; 
  constexpr double kSliderKp = 800.0;
  constexpr double kSliderKd = 5.0e-3;

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

  plant.Finalize();

  auto* slider_source =
      builder.AddSystem<SliderStrokeSource>(kSliderAmplitude, kDriveFrequencyHz);
  builder.Connect(slider_source->get_output_port(),
                  plant.get_desired_state_input_port(model_instance));

  auto* wing_aero = builder.AddSystem<WingAeromechanics>(
      plant, kAngularAccelerationSamplePeriod,
      kAngularAccelerationFilterTimeConstant);
  builder.Connect(plant.get_state_output_port(),
                  wing_aero->get_input_port(0));
  builder.Connect(wing_aero->get_output_port(0),
                  plant.get_applied_spatial_force_input_port());

  drake::lcm::DrakeLcm lcm;
  drake::geometry::DrakeVisualizerParams visualizer_params;
  visualizer_params.publish_period = kVisualizerPublishPeriod;
  drake::geometry::DrakeVisualizerd::AddToBuilder(
      &builder, scene_graph, &lcm, visualizer_params);

  auto diagram = builder.Build();
  drake::systems::Simulator<double> simulator(*diagram);
  simulator.set_target_realtime_rate(kTargetRealtimeRate);
  simulator.Initialize();

  std::cout << "RoboBee constrained linkage model is being simulated and "
               "published on LCM for Meldis.\n"
            << "Start Meldis before or after this process:\n"
            << "  bazel run @drake//tools:meldis -- --open-window\n\n"
            << "Aeromechanical moments are being logged to:\n"
            << "  " << kMomentLogPath << "\n\n"
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
            << "  target realtime rate: " << kTargetRealtimeRate << "\n\n"
            << "The slider joints are driven by finite-gain joint actuators "
               "tracking a sinusoidal desired position and velocity; "
               "the transmission loops are closed with MultibodyPlant weld "
               "constraints, not per-frame IK.\n"
            << "Press Ctrl-C here to stop publishing.\n";

  drake::geometry::DrakeVisualizerd::DispatchLoadMessage(scene_graph, &lcm);

  std::ofstream moment_log(kMomentLogPath);
  if (!moment_log) {
    throw std::runtime_error("Could not open aeromechanical moment CSV log.");
  }
  WriteMomentCsvHeader(&moment_log);

  double next_time = 0.0;
  double next_moment_log_time = 0.0;
  double next_moment_log_flush_time = kMomentLogFlushPeriod;
  while (true) {
    next_time += kCommandPeriod;
    simulator.AdvanceTo(next_time);
    if (next_time + 0.5 * kCommandPeriod >= next_moment_log_time) {
      WriteMomentCsvRow(&moment_log, next_time, wing_aero->latest_moments());
      next_moment_log_time += kMomentLogPeriod;
    }
    if (next_time + 0.5 * kCommandPeriod >= next_moment_log_flush_time) {
      moment_log.flush();
      next_moment_log_flush_time += kMomentLogFlushPeriod;
    }
  }

  return 0;
}
