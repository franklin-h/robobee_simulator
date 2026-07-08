#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>

#include "drake/geometry/drake_visualizer.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/joint_actuator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"

#include "apps/robobee_assembly_loader.h"

namespace {

constexpr char kPackageName[] = "robobee_assembly";
constexpr char kPackagePath[] = "models/robobee_assembly";
constexpr double kPi = 3.14159265358979323846;

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
    const double v = stroke_amplitude_ * drive_omega_ * std::cos(drive_omega_ * t);

    // Actuators are added below in this order: slider_1, then slider_2.
    Eigen::VectorBlock<Eigen::VectorXd> y = output->get_mutable_value();
    y << q, q, v, v;
  }

  double stroke_amplitude_{};
  double drive_omega_{};
};

void RegisterRoboBeePackage(drake::multibody::Parser* parser) {
  parser->package_map().Add(kPackageName, kPackagePath);
}

drake::math::RigidTransformd CalcDefaultPoseOfLoopFrameInRealLinkFrame(
    const std::string& loop_body_name, const std::string& real_body_name) {
  drake::multibody::MultibodyPlant<double> plant(0.0);
  drake::multibody::Parser parser(&plant);
  RegisterRoboBeePackage(&parser);
  robobee_sim::AddRoboBeeAssemblyModels(&parser);
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("root"));
  plant.Finalize();

  std::unique_ptr<drake::systems::Context<double>> context =
      plant.CreateDefaultContext();
  return plant.CalcRelativeTransform(*context,
                                     plant.GetFrameByName(real_body_name),
                                     plant.GetFrameByName(loop_body_name));
}

// Recover, in the default CAD pose, the loop-closure pivot point expressed in
// an arbitrary body frame together with that body's local direction of the
// loop revolute axis (the exported hinge axis is +Y in the helper body frame).
void CalcDefaultLoopFramePointAndAxis(
    const std::string& loop_body_name, const std::string& body_name,
    Eigen::Vector3d* p_BL, Eigen::Vector3d* axis_B) {
  drake::multibody::MultibodyPlant<double> plant(0.0);
  drake::multibody::Parser parser(&plant);
  RegisterRoboBeePackage(&parser);
  robobee_sim::AddRoboBeeAssemblyModels(&parser);
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

// Close a transmission four-bar by directly re-creating, between two real
// (inertia-bearing) bodies, the revolute pin that the URDF opened into the
// helper body. `anchor_body_name` is the tree-side link carrying that pivot
// (transmission_link_1) and `real_link_body_name` is the coupler
// (transmission_link_2); the helper `loop_body_name` supplies the pivot point
// and axis from the default CAD pose. Two ball constraints separated along the
// axis pin the shared pivot and align it, leaving one rotational DOF. Coupling
// two real bodies this way transmits slider motion into hinge rotation far more
// stiffly than the compliant weld propagating through the near-massless helper.
// All bodies are named exactly as in the URDF, so no joint names are modified.
void AddLoopClosureBallConstraints(
    drake::multibody::MultibodyPlant<double>* plant,
    const std::string& loop_body_name, const std::string& anchor_body_name,
    const std::string& real_link_body_name) {
  Eigen::Vector3d p_AL;
  Eigen::Vector3d axis_A;
  CalcDefaultLoopFramePointAndAxis(loop_body_name, anchor_body_name, &p_AL,
                                   &axis_A);

  Eigen::Vector3d p_RL;
  Eigen::Vector3d axis_R;
  CalcDefaultLoopFramePointAndAxis(loop_body_name, real_link_body_name, &p_RL,
                                   &axis_R);

  constexpr double kAxisPointOffset = 1.0e-4;
  plant->AddBallConstraint(plant->GetBodyByName(anchor_body_name), p_AL,
                           plant->GetBodyByName(real_link_body_name), p_RL);
  plant->AddBallConstraint(plant->GetBodyByName(anchor_body_name),
                           p_AL + kAxisPointOffset * axis_A.normalized(),
                           plant->GetBodyByName(real_link_body_name),
                           p_RL + kAxisPointOffset * axis_R.normalized());
}

}  // namespace

int main() {
  drake::systems::DiagramBuilder<double> builder;

  constexpr double kPlantTimeStep = 1e-3;
  auto [plant, scene_graph] =
      drake::multibody::AddMultibodyPlantSceneGraph(&builder, kPlantTimeStep);
  plant.set_discrete_contact_approximation(
      drake::multibody::DiscreteContactApproximation::kSap);

  drake::multibody::Parser parser(&plant);
  RegisterRoboBeePackage(&parser);
  const std::vector<drake::multibody::ModelInstanceIndex> model_instances =
      robobee_sim::AddRoboBeeAssemblyModels(&parser);
  if (model_instances.size() != 1) {
    throw std::runtime_error("Expected exactly one RoboBee model instance.");
  }
  const drake::multibody::ModelInstanceIndex model_instance =
      model_instances.front();

  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("root"));

  // Close the transmission loops while preserving the exported URDF names and
  // topology. The helper body is the non-tree side of the loop; welding it to
  // the real link lets Drake enforce the closure constraint at runtime.
  AddLoopClosureWeldConstraint(&plant,
                               "transmission_link_2__1__loop_closure",
                               "transmission_link_2");
  AddLoopClosureWeldConstraint(&plant,
                               "transmission_right_link_2__1__loop_closure",
                               "transmission_right_link_2");
  AddLoopClosureBallConstraints(&plant,
                                "transmission_link_2__1__loop_closure",
                                "transmission_link_1", "transmission_link_2");
  AddLoopClosureBallConstraints(&plant,
                                "transmission_right_link_2__1__loop_closure",
                                "transmission_right_link_1",
                                "transmission_right_link_2");

  const auto& right_slider_actuator = plant.AddJointActuator(
      "right_slider_drive", plant.GetJointByName("slider_1"),
      0.02 /* effort_limit_N */);
  plant.get_mutable_joint_actuator(right_slider_actuator.index())
      .set_controller_gains({10.0 /* p */, 0.002 /* d */});

  const auto& left_slider_actuator = plant.AddJointActuator(
      "left_slider_drive", plant.GetJointByName("slider_2"),
      0.02 /* effort_limit_N */);
  plant.get_mutable_joint_actuator(left_slider_actuator.index())
      .set_controller_gains({10.0 /* p */, 0.002 /* d */});

  plant.Finalize();

  constexpr double kSliderAmplitude = 0.00020;  // 0.6 mm peak-to-peak.
  constexpr double kDriveFrequencyHz = 0.5;
  auto* slider_source =
      builder.AddSystem<SliderStrokeSource>(kSliderAmplitude, kDriveFrequencyHz);
  builder.Connect(slider_source->get_output_port(),
                  plant.get_desired_state_input_port(model_instance));

  drake::lcm::DrakeLcm lcm;
  drake::geometry::DrakeVisualizerd::AddToBuilder(&builder, scene_graph, &lcm);


//   drake::geometry::DrakeVisualizerParams visualizer_params;
// visualizer_params.publish_period = 1.0 / 64.0;

//     drake::geometry::DrakeVisualizerd::AddToBuilder(
//     &builder, scene_graph, &lcm, visualizer_params);

  auto diagram = builder.Build();
  drake::systems::Simulator<double> simulator(*diagram);
  simulator.set_target_realtime_rate(1);
  simulator.Initialize();

  std::cout << "RoboBee constrained linkage model is being simulated and "
               "published on LCM for Meldis.\n"
            << "Start Meldis before or after this process:\n"
            << "  bazel run @drake//tools:meldis -- --open-window\n\n"
            << "The slider joints are driven by plant-level PD actuators; "
               "the transmission loops are closed with MultibodyPlant weld "
               "constraints, not per-frame IK.\n"
            << "Press Ctrl-C here to stop publishing.\n";

  drake::geometry::DrakeVisualizerd::DispatchLoadMessage(scene_graph, &lcm);

  double next_time = 0.0;
  auto last_load_message = std::chrono::steady_clock::now();
  while (true) {
    next_time += 1.0 / 60.0;
    simulator.AdvanceTo(next_time);

    const auto now = std::chrono::steady_clock::now();
    if (now - last_load_message >= std::chrono::seconds(1)) {
      drake::geometry::DrakeVisualizerd::DispatchLoadMessage(scene_graph, &lcm);
      last_load_message = now;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  return 0;
}
