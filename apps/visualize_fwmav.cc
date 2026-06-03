#include <chrono>
#include <iostream>
#include <thread>

#include "drake/geometry/drake_visualizer.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

namespace {

void AddLocalPackages(drake::multibody::Parser* parser) {
  parser->package_map().Add("fwmav_asy", "models/fwmav_asy");
  parser->package_map().Add("wing_asy", "models/wing_asy");
  parser->package_map().Add("wing_asy_right", "models/wing_asy_right");
}

}  // namespace

int main() {
  drake::systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] =
      drake::multibody::AddMultibodyPlantSceneGraph(&builder, 0.0);

  drake::multibody::Parser parser(&plant);
  AddLocalPackages(&parser);
  parser.AddModelsFromUrl("package://fwmav_asy/urdf/fwmav_asy.urdf");

  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("fuselage"));
  plant.Finalize();

  drake::geometry::DrakeVisualizerd::AddToBuilder(&builder, scene_graph);

  auto diagram = builder.Build();
  drake::systems::Simulator<double> simulator(*diagram);
  simulator.Initialize();

  std::cout << "FWMAV geometry is being published on LCM for Meldis.\n"
            << "In another terminal, run:\n"
            << "  bazel run @drake//tools:meldis -- --open-window\n\n"
            << "Press Ctrl-C here to stop publishing.\n";

  while (true) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  return 0;
}
