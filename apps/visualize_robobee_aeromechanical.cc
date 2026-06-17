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
#include "drake/lcm/drake_lcm.h"
#include "drake/math/rigid_transform.h"
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
  Eigen::Vector3d reference_chord_axis_W{Eigen::Vector3d::Zero()};
};

struct WingAngularHistory {
  bool initialized{false};
  double time_s{};
  Eigen::Vector3d omega_B{Eigen::Vector3d::Zero()};
  Eigen::Vector3d omega_dot_B{Eigen::Vector3d::Zero()};
};

class WingAeromechanics final : public drake::systems::LeafSystem<double> {
 public:
  explicit WingAeromechanics(const drake::multibody::MultibodyPlant<double>& plant)
      : plant_(plant),
        plant_context_(plant.CreateDefaultContext()),
        wings_({WingAeroConfig{"hinge_left_wing",
                                &robobee::kLeftWingAeromechanicalConstants,
                                Eigen::Vector3d::UnitX(),
                                Eigen::Vector3d::UnitY(),
                                Eigen::Vector3d::UnitZ(), 1.0},
                WingAeroConfig{"hinge_right_wing",
                                &robobee::kRightWingAeromechanicalConstants,
                                Eigen::Vector3d::UnitX(),
                                Eigen::Vector3d::UnitY(),
                                Eigen::Vector3d::UnitZ(), 1.0}}) {
    for (WingAeroConfig& wing : wings_) {
      const auto& body = plant_.GetBodyByName(wing.body_name);
      const Eigen::Matrix3d R_WB0 =
          body.EvalPoseInWorld(*plant_context_).rotation().matrix();
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
    if (dt > 1.0e-12) {
      history.omega_dot_B = (omega_B - history.omega_B) / dt;
      history.omega_B = omega_B;
      history.time_s = time_s;
    }
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
      output->push_back(CalcWingSpatialForce(wings_[i], i, context.get_time()));
    }
  }

  drake::multibody::ExternallyAppliedSpatialForce<double> CalcWingSpatialForce(
      const WingAeroConfig& wing, int wing_index, double time_s) const {
    const auto& constants = *wing.constants;
    const auto& body = plant_.GetBodyByName(wing.body_name);
    latest_moments_.at(wing_index) = {};

    const drake::math::RigidTransformd& X_WB =
        body.EvalPoseInWorld(*plant_context_);
    const auto& V_WB = body.EvalSpatialVelocityInWorld(*plant_context_);
    if (!X_WB.GetAsMatrix4().array().isFinite().all() ||
        !IsFinite(V_WB.translational()) || !IsFinite(V_WB.rotational())) {
      return MakeZeroForce(body);
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
      return MakeZeroForce(body);
    }

    const Eigen::Vector3d wind_W = Eigen::Vector3d::Zero();
    const auto& stations = constants.blade_stations;
    robobee::WingMomentInput moment_input;
    moment_input.station_flows.reserve(stations.size());

    for (int i = 0; i < static_cast<int>(stations.size()); ++i) {
      const auto& station = stations[i];
      if (!std::isfinite(station.r_m) ||
          !std::isfinite(station.y_hinge_to_mid_chord_hat) ||
          !std::isfinite(station.chord_m)) {
        moment_input.station_flows.push_back({});
        continue;
      }

      const Eigen::Vector3d p_BoP_B =
          station.r_m * wing.span_axis_B +
          station.y_hinge_to_mid_chord_hat * constants.mean_chord_cbar_m *
              wing.chord_axis_B;
      const Eigen::Vector3d p_BoP_W = R_WB * p_BoP_B;
      const Eigen::Vector3d v_P_W =
          V_WB.translational() + V_WB.rotational().cross(p_BoP_W);
      const Eigen::Vector3d v_rel_W = v_P_W - wind_W;
      if (!IsFinite(v_rel_W)) {
        moment_input.station_flows.push_back({});
        continue;
      }
      moment_input.station_flows.push_back(
          {v_rel_W.dot(chord_axis_W), v_rel_W.dot(normal_axis_W)});
    }

    const Eigen::Vector3d omega_B = R_WB.transpose() * V_WB.rotational();
    const Eigen::Vector3d omega_dot_B =
        EstimateAngularAccelerationInBody(wing_index, time_s, omega_B);
    if (!IsFinite(omega_B) || !IsFinite(omega_dot_B)) {
      return MakeZeroForce(body);
    }

    moment_input.pitch_angle_psi_rad =
        CalcPitchAngleAboutSpan(wing, span_axis_W, chord_axis_W);
    moment_input.omega_x_rad_s = omega_B.x();
    moment_input.omega_y_rad_s = omega_B.y();
    moment_input.omega_z_rad_s = omega_B.z();
    moment_input.omega_dot_x_rad_s2 = omega_dot_B.x();
    moment_input.omega_dot_y_rad_s2 = omega_dot_B.y();

    const robobee::WingMomentComponents moments =
        robobee::CalcAeromechanicalMoments(constants, moment_input);
    latest_moments_.at(wing_index) = moments;

    drake::multibody::ExternallyAppliedSpatialForce<double> force;
    force.body_index = body.index();
    force.p_BoBq_B = Eigen::Vector3d::Zero();
    force.F_Bq_W = drake::multibody::SpatialForce<double>(
        wing.moment_sign * moments.applied_total_Nm * span_axis_W,
        Eigen::Vector3d::Zero());
    return force;
  }

  static drake::multibody::ExternallyAppliedSpatialForce<double> MakeZeroForce(
      const drake::multibody::RigidBody<double>& body) {
    drake::multibody::ExternallyAppliedSpatialForce<double> force;
    force.body_index = body.index();
    force.p_BoBq_B = Eigen::Vector3d::Zero();
    force.F_Bq_W = drake::multibody::SpatialForce<double>(
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    return force;
  }

  const drake::multibody::MultibodyPlant<double>& plant_;
  mutable std::unique_ptr<drake::systems::Context<double>> plant_context_;
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

void WriteMomentCsvHeader(std::ostream* output) {
  *output
      << "time_s,"
      << "left_alpha_rad,left_aero_Nm,left_rot_Nm,left_added_Nm,left_hinge_Nm,"
         "left_total_Nm,left_applied_Nm,"
      << "right_alpha_rad,right_aero_Nm,right_rot_Nm,right_added_Nm,right_hinge_Nm,"
         "right_total_Nm,right_applied_Nm\n";
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
          << left.aerodynamic_Nm << ',' << left.rotational_damping_Nm << ','
          << left.added_mass_Nm << ',' << left.hinge_Nm << ','
          << left.total_Nm << ',' << left.applied_total_Nm << ','
          << right.angle_of_attack_alpha_rad << ','
          << right.aerodynamic_Nm << ',' << right.rotational_damping_Nm << ','
          << right.added_mass_Nm << ',' << right.hinge_Nm << ','
          << right.total_Nm << ',' << right.applied_total_Nm << '\n';
}

}  // namespace

int main() {
  drake::systems::DiagramBuilder<double> builder;

  constexpr double kPlantTimeStep = 5e-5;
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
  constexpr double kSliderEffortLimitN = 10.0; 
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

  plant.Finalize();

  constexpr double kSliderAmplitude = 0.00020;  // 0.4 mm peak-to-peak.
  constexpr double kDriveFrequencyHz = 100;
  constexpr double kCommandPeriod = kPlantTimeStep;
  constexpr double kMomentLogPeriod = 5e-4;
  constexpr char kMomentLogPath[] = "/tmp/aeromechanical_moments.csv";

  auto* slider_source =
      builder.AddSystem<SliderStrokeSource>(kSliderAmplitude, kDriveFrequencyHz);
  builder.Connect(slider_source->get_output_port(),
                  plant.get_desired_state_input_port(model_instance));

  auto* wing_aero = builder.AddSystem<WingAeromechanics>(plant);
  builder.Connect(plant.get_state_output_port(),
                  wing_aero->get_input_port(0));
  builder.Connect(wing_aero->get_output_port(0),
                  plant.get_applied_spatial_force_input_port());

  drake::lcm::DrakeLcm lcm;
  drake::geometry::DrakeVisualizerParams visualizer_params;
  visualizer_params.publish_period = 5e-4;
  drake::geometry::DrakeVisualizerd::AddToBuilder(
      &builder, scene_graph, &lcm, visualizer_params);

  auto diagram = builder.Build();
  drake::systems::Simulator<double> simulator(*diagram);
  simulator.set_target_realtime_rate(0.02);
  simulator.Initialize();

  std::cout << "RoboBee constrained linkage model is being simulated and "
               "published on LCM for Meldis.\n"
            << "Start Meldis before or after this process:\n"
            << "  bazel run @drake//tools:meldis -- --open-window\n\n"
            << "Aeromechanical moments are being logged to:\n"
            << "  " << kMomentLogPath << "\n\n"
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
  while (true) {
    next_time += kCommandPeriod;
    simulator.AdvanceTo(next_time);
    if (next_time + 0.5 * kCommandPeriod >= next_moment_log_time) {
      WriteMomentCsvRow(&moment_log, next_time, wing_aero->latest_moments());
      moment_log.flush();
      next_moment_log_time += kMomentLogPeriod;
    }
  }

  return 0;
}
