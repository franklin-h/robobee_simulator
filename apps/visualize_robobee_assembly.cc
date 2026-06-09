#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

#include "drake/geometry/drake_visualizer.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/solve.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

/* TODO: 
It seems that transmission_link_2 is not rotating; Drake happily breaks the revolute joint between link_2 and hinge, while keeping transmission_link_2 completely in line with transmission_link_1, when transmission_link_2 should also be on some arc like path. 
*/
namespace {

  // Adds position and orientation constraints to `ik` to hold `frame` at the
  // specified pose `X_WF` in the world frame.
void AddWorldPoseConstraint(
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::multibody::Frame<double>& frame,
    const drake::math::RigidTransformd& X_WF,
    drake::multibody::InverseKinematics* ik) {
  constexpr double kPositionTolerance = 1e-8;
  ik->AddPositionConstraint(
      frame, Eigen::Vector3d::Zero(), plant.world_frame(),
      X_WF.translation() - Eigen::Vector3d::Constant(kPositionTolerance),
      X_WF.translation() + Eigen::Vector3d::Constant(kPositionTolerance));
  ik->AddOrientationConstraint(plant.world_frame(), X_WF.rotation(), frame,
                               drake::math::RotationMatrixd(), 1e-6);
}

// 
void AddInitialCoincidenceConstraint(
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::systems::Context<double>& plant_context,
    const drake::multibody::Frame<double>& point_frame,
    const drake::multibody::Frame<double>& target_frame,
    drake::multibody::InverseKinematics* ik) {
  const drake::math::RigidTransformd X_TP =
      plant.CalcRelativeTransform(plant_context, target_frame, point_frame);
  const Eigen::Vector3d p_PQ = X_TP.inverse().translation();
  constexpr double kTolerance = 1e-7;
  ik->AddPositionConstraint(
      point_frame, p_PQ, target_frame,
      Eigen::Vector3d::Constant(-kTolerance),
      Eigen::Vector3d::Constant(kTolerance));
}

void AddInitialAxisAlignmentConstraint(
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::systems::Context<double>& plant_context,
    const drake::multibody::Frame<double>& axis_frame,
    const drake::multibody::Frame<double>& target_frame,
    drake::multibody::InverseKinematics* ik) {
  const drake::math::RigidTransformd X_TA =
      plant.CalcRelativeTransform(plant_context, target_frame, axis_frame);
  const Eigen::Vector3d axis_A(0.0, -1.0, 0.0);
  const Eigen::Vector3d axis_T = X_TA.rotation() * axis_A;
  constexpr double kAxisTolerance = 1e-4;
  ik->AddAngleBetweenVectorsConstraint(axis_frame, axis_A, target_frame, axis_T,
                                       0.0, kAxisTolerance);
}

bool SolveWingKinematics(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* plant_context,
    const drake::math::RigidTransformd& X_WLeftBase,
    const drake::math::RigidTransformd& X_WRightBase, double slider_position,
    Eigen::VectorXd* q_guess) {
  drake::multibody::InverseKinematics ik(plant, plant_context, false);
  auto* prog = ik.get_mutable_prog();
  const auto& q = ik.q();

  const int right_slider_index =
      plant.GetJointByName("slider_1").position_start();
  const int left_slider_index =
      plant.GetJointByName("slider_2").position_start();
  prog->AddBoundingBoxConstraint(slider_position, slider_position,
                                 q.segment<1>(right_slider_index));
  prog->AddBoundingBoxConstraint(slider_position, slider_position,
                                 q.segment<1>(left_slider_index));

  // Define the base frame of the left and right transmission kinematic chains. 
  AddWorldPoseConstraint(plant, plant.GetFrameByName("transmission_base"),
                         X_WLeftBase, &ik);
  AddWorldPoseConstraint(
      plant, plant.GetFrameByName("transmission_right_link_base"), X_WRightBase,
      &ik);

  // URDF export forces open kinematic tree. 
  AddInitialCoincidenceConstraint(
      plant, *plant_context,
      plant.GetFrameByName("transmission_link_2__1__loop_closure"),
      plant.GetFrameByName("transmission_link_2"), &ik);
  AddInitialAxisAlignmentConstraint(
      plant, *plant_context,
      plant.GetFrameByName("transmission_link_2__1__loop_closure"),
      plant.GetFrameByName("transmission_link_2"), &ik);
  AddInitialCoincidenceConstraint(
      plant, *plant_context,
      plant.GetFrameByName("transmission_right_link_2__1__loop_closure"),
      plant.GetFrameByName("transmission_right_link_2"), &ik);
  AddInitialAxisAlignmentConstraint(
      plant, *plant_context,
      plant.GetFrameByName("transmission_right_link_2__1__loop_closure"),
      plant.GetFrameByName("transmission_right_link_2"), &ik);

  prog->AddQuadraticErrorCost(Eigen::MatrixXd::Identity(q.size(), q.size()),
                              *q_guess, q);
  prog->SetInitialGuess(q, *q_guess);

  const drake::solvers::MathematicalProgramResult result =
      drake::solvers::Solve(ik.prog());
  if (!result.is_success()) {
    return false;
  }

  *q_guess = result.GetSolution(q);
  plant.SetPositions(plant_context, *q_guess);
  return true;
}

}  // namespace

int main() {
  drake::systems::DiagramBuilder<double> builder;

  // set plant equal to the multibody plant
  auto [plant, scene_graph] =
      drake::multibody::AddMultibodyPlantSceneGraph(&builder, 0.0);
  drake::lcm::DrakeLcm lcm;

  drake::multibody::Parser parser(&plant);
  parser.package_map().Add("robobee_assembly", "models/robobee_assembly");
  parser.AddModelsFromUrl(
      "package://robobee_assembly/urdf/"
      "robobee_assembly.urdf");

  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("root")); // anchor robot rood to world 
  plant.Finalize();

  drake::geometry::DrakeVisualizerd::AddToBuilder(&builder, scene_graph, &lcm);

  auto diagram = builder.Build();
  drake::systems::Simulator<double> simulator(*diagram);
  simulator.Initialize();
  auto& root_context = simulator.get_mutable_context();
  auto& plant_context = plant.GetMyMutableContextFromRoot(&root_context);
  Eigen::VectorXd q_guess = plant.GetPositions(plant_context);

  // pose of the left transmission base frame in world frame. 
  const drake::math::RigidTransformd X_WLeftBase =
      plant.CalcRelativeTransform(plant_context, plant.world_frame(),
                                  plant.GetFrameByName("transmission_base"));
  const drake::math::RigidTransformd X_WRightBase =
      plant.CalcRelativeTransform(
          plant_context, plant.world_frame(),
          plant.GetFrameByName("transmission_right_link_base"));

  std::cout << "Robobee assembly geometry is being published on LCM "
               "for Meldis.\n"
            << "Start Meldis before or after this process:\n"
            << "  bazel run @drake//tools:meldis -- --open-window\n\n"
            << "Driving both transmission link 1 inputs through a "
               "0.2 mm peak-to-peak stroke and solving both "
               "transmission/hinge/wing chains.\n"
            << "Press Ctrl-C here to stop publishing.\n";

  drake::geometry::DrakeVisualizerd::DispatchLoadMessage(scene_graph, &lcm);
  const auto start_time = std::chrono::steady_clock::now();
  auto last_load_message = start_time;
  constexpr double kSliderAmplitude = 0.00020;  // 0.2 mm peak-to-peak.
  constexpr double kDriveFrequencyHz = 0.5;
  constexpr double kPi = 3.14159265358979323846;

  while (true) {
    const auto now = std::chrono::steady_clock::now();
    const double time =
        std::chrono::duration<double>(now - start_time).count();
        
    const double slider_position =
        kSliderAmplitude * std::sin(2.0 * kPi * kDriveFrequencyHz * time);
    if (!SolveWingKinematics(plant, &plant_context, X_WLeftBase, X_WRightBase,
                             slider_position, &q_guess)) {
      std::cerr << "Wing kinematic solve failed for slider inputs = "
                << slider_position << " m\n";
    }

    if (now - last_load_message >= std::chrono::seconds(1)) {
      drake::geometry::DrakeVisualizerd::DispatchLoadMessage(scene_graph, &lcm);
      last_load_message = now;
    }
    diagram->ForcedPublish(root_context);
    std::this_thread::sleep_for(std::chrono::milliseconds(33));
  }

  return 0;
}
