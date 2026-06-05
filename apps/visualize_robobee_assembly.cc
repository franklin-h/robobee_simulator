#include <chrono>
#include <iostream>
#include <thread>

#include "drake/geometry/drake_visualizer.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

int main() {
  drake::systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] =
      drake::multibody::AddMultibodyPlantSceneGraph(&builder, 0.0);
  drake::lcm::DrakeLcm lcm;

  drake::multibody::Parser parser(&plant);
  parser.package_map().Add("robobee_assembly", "models/robobee_assembly");
  parser.AddModelsFromUrl(
      "package://robobee_assembly/urdf/"
      "robobee_assembly.urdf");

  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("root"));
  plant.Finalize();

  drake::geometry::DrakeVisualizerd::AddToBuilder(&builder, scene_graph, &lcm);

  auto diagram = builder.Build();
  drake::systems::Simulator<double> simulator(*diagram);
  simulator.Initialize();
  auto& root_context = simulator.get_mutable_context();

  std::cout << "Robobee assembly geometry is being published on LCM "
               "for Meldis.\n"
            << "Start Meldis before or after this process:\n"
            << "  bazel run @drake//tools:meldis -- --open-window\n\n"
            << "Republishing the static geometry load message once per second.\n"
            << "Press Ctrl-C here to stop publishing.\n";

  while (true) {
    drake::geometry::DrakeVisualizerd::DispatchLoadMessage(scene_graph, &lcm);
    diagram->ForcedPublish(root_context);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  return 0;
}
