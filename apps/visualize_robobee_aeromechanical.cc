#include <algorithm>
#include <cmath>
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
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"

#include "aeromechanical_wing_constants.h"

namespace {

constexpr char kPackageName[] = "robobee_assembly";
constexpr char kPackagePath[] = "models/robobee_assembly";
constexpr char kAssemblyUrl[] =
    "package://robobee_assembly/urdf/robobee_assembly.urdf";
constexpr double kPi = 3.14159265358979323846;
constexpr double kAirDensity = 1.225;   // kg/m^3, sea-level standard air.
constexpr double kClMax = 1.8;
constexpr double kCdMax = 3.4;
constexpr double kCd0 = 0.4;
constexpr double kMaxAeroPitchMomentNm = 2.0e-11;

/* 
TODO: Move the constants above into aeromechanical_wing_constants.h. Modify the html to include places where those can be entered. 
*/

/* 
TODO: Have a GUI interface with simulation parameters for a section. And also a section (placeholder for now) which will have robobee parameters. 
*/

struct WingAeroConfig {
  std::string body_name;
  Eigen::Vector3d span_axis_B;
  Eigen::Vector3d chord_axis_B;
  Eigen::Vector3d normal_axis_B;
  double moment_sign{1.0};
};

class WingAeromechanics final : public drake::systems::LeafSystem<double> {
 public:
  explicit WingAeromechanics(const drake::multibody::MultibodyPlant<double>& plant)
      : plant_(plant),
        plant_context_(plant.CreateDefaultContext()),
        wings_({WingAeroConfig{"hinge_wing", Eigen::Vector3d::UnitX(),
                                Eigen::Vector3d::UnitY(),
                                Eigen::Vector3d::UnitZ(), 1.0},
                WingAeroConfig{"hinge_right_wing", Eigen::Vector3d::UnitX(),
                                Eigen::Vector3d::UnitY(),
                                Eigen::Vector3d::UnitZ(), 1.0}}) {
    this->DeclareVectorInputPort("plant_state", plant.num_multibody_states());
    this->DeclareAbstractOutputPort("spatial_forces",
                                    &WingAeromechanics::CalcSpatialForces);
  }

 private:
  static double Sign(double value) {
    if (value > 0.0) return 1.0;
    if (value < 0.0) return -1.0;
    return 0.0;
  }

  static bool IsFinite(const Eigen::Vector3d& value) {
    return value.array().isFinite().all();
  }

  static double LiftCoefficient(double alpha) {
    return kClMax * std::sin(2.0 * alpha);
  }

  static double DragCoefficient(double alpha) {
    return 0.5 * (kCdMax + kCd0) -
           0.5 * (kCdMax - kCd0) * std::cos(2.0 * alpha);
  }

  static double NormalForceCoefficient(double alpha) {
    return LiftCoefficient(alpha) * std::cos(alpha) +
           DragCoefficient(alpha) * std::sin(alpha);
  }

  static double StationWidth(int index) {
    const auto& stations = robobee::kLeftWingAeromechanicalConstants.blade_stations;
    if (stations.size() < 2) return 0.0;
    if (index == 0) return 0.5 * (stations[1].r_m - stations[0].r_m);
    if (index == static_cast<int>(stations.size()) - 1) {
      return 0.5 * (stations[index].r_m - stations[index - 1].r_m);
    }
    return 0.5 * (stations[index + 1].r_m - stations[index - 1].r_m);
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

    for (const WingAeroConfig& wing : wings_) {
      output->push_back(CalcWingSpatialForce(wing));
    }
  }

  drake::multibody::ExternallyAppliedSpatialForce<double> CalcWingSpatialForce(
      const WingAeroConfig& wing) const {
    const auto& constants = robobee::kLeftWingAeromechanicalConstants;
    const auto& body = plant_.GetBodyByName(wing.body_name);

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

    double pitch_moment_Nm = 0.0;
    const Eigen::Vector3d wind_W = Eigen::Vector3d::Zero();
    const auto& stations = constants.blade_stations;

    for (int i = 0; i < static_cast<int>(stations.size()); ++i) {
      const auto& station = stations[i];
      const double dr = StationWidth(i);
      if (!std::isfinite(dr) || dr <= 0.0 ||
          !std::isfinite(station.r_m) || !std::isfinite(station.chord_m) ||
          !std::isfinite(station.y_hinge_to_mid_chord_hat) ||
          !std::isfinite(station.leading_edge_q_m) ||
          station.chord_m <= 0.0) {
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
      if (!IsFinite(v_rel_W)) continue;

      const double v_chord = v_rel_W.dot(chord_axis_W);
      const double v_normal = v_rel_W.dot(normal_axis_W);
      const double speed_squared = v_chord * v_chord + v_normal * v_normal;
      if (!std::isfinite(speed_squared) || speed_squared <= 1.0e-16) continue;

      const double alpha = std::atan2(-v_normal, -v_chord);
      const double c_n = NormalForceCoefficient(alpha);
      const double d_force_n =
          0.5 * kAirDensity * speed_squared * c_n * station.chord_m * dr;
      const double d_cp_hat = 0.82 / kPi * std::abs(alpha) + 0.05;
      const double y_le =
          constants.y_r_hat_by_cbar * constants.mean_chord_cbar_m +
          station.leading_edge_q_m;
      const double y_cp = y_le - station.chord_m * d_cp_hat;
      if (!std::isfinite(alpha) || !std::isfinite(c_n) ||
          !std::isfinite(d_force_n) || !std::isfinite(y_cp)) {
        continue;
      }
      pitch_moment_Nm += -Sign(alpha) * y_cp * d_force_n;
    }

    if (!std::isfinite(pitch_moment_Nm)) {
      return MakeZeroForce(body);
    }
    pitch_moment_Nm = std::clamp(pitch_moment_Nm, -kMaxAeroPitchMomentNm,
                                 kMaxAeroPitchMomentNm);

    drake::multibody::ExternallyAppliedSpatialForce<double> force;
    force.body_index = body.index();
    force.p_BoBq_B = Eigen::Vector3d::Zero();
    force.F_Bq_W = drake::multibody::SpatialForce<double>(
        wing.moment_sign * pitch_moment_Nm * span_axis_W,
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

void SetSliderStrokeCommand(
    const drake::multibody::PrismaticJoint<double>& right_slider,
    const drake::multibody::PrismaticJoint<double>& left_slider,
    drake::systems::Context<double>* plant_context, double time,
    double stroke_amplitude, double drive_omega) {
  const double q = stroke_amplitude * std::sin(drive_omega * time);

  right_slider.set_translation(plant_context, q);
  right_slider.set_translation_rate(plant_context, 0.0);
  left_slider.set_translation(plant_context, q);
  left_slider.set_translation_rate(plant_context, 0.0);
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

  const auto& right_slider =
      dynamic_cast<const drake::multibody::PrismaticJoint<double>&>(
          plant.GetJointByName("slider_1"));
  const auto& left_slider =
      dynamic_cast<const drake::multibody::PrismaticJoint<double>&>(
          plant.GetJointByName("slider_2"));

  plant.Finalize();

  constexpr double kSliderAmplitude = 0.00020;  // 0.4 mm peak-to-peak.
  constexpr double kDriveFrequencyHz = 100;
  constexpr double kDriveOmega = 2.0 * kPi * kDriveFrequencyHz;
  constexpr double kCommandPeriod = kPlantTimeStep;

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
  auto& root_context = simulator.get_mutable_context();
  auto& plant_context =
      diagram->GetMutableSubsystemContext(plant, &root_context);
  SetSliderStrokeCommand(right_slider, left_slider, &plant_context, 0.0,
                         kSliderAmplitude, kDriveOmega);
  simulator.set_target_realtime_rate(0.02);
  simulator.Initialize();

  std::cout << "RoboBee constrained linkage model is being simulated and "
               "published on LCM for Meldis.\n"
            << "Start Meldis before or after this process:\n"
            << "  bazel run @drake//tools:meldis -- --open-window\n\n"
            << "The slider joints are kinematically commanded by writing "
               "their translation before each plant step, with slider rates "
               "zeroed; "
               "the transmission loops are closed with MultibodyPlant weld "
               "constraints, not per-frame IK.\n"
            << "Press Ctrl-C here to stop publishing.\n";

  drake::geometry::DrakeVisualizerd::DispatchLoadMessage(scene_graph, &lcm);

  double next_time = 0.0;
  while (true) {
    SetSliderStrokeCommand(right_slider, left_slider, &plant_context, next_time,
                           kSliderAmplitude, kDriveOmega);
    next_time += kCommandPeriod;
    simulator.AdvanceTo(next_time);
  }

  return 0;
}
