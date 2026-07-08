#pragma once

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/model_instance.h"

namespace robobee_sim {

inline std::string ReadTextFileOrThrow(const std::string& path) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("Could not open " + path);
  }
  std::ostringstream buffer;
  buffer << input.rdbuf();
  return buffer.str();
}

inline std::string LoadRoboBeeAssemblyUrdfForDrake() {
  // Load the Onshape export verbatim. The exporter opens each closed four-bar
  // by keeping the visual coupler link (transmission_link_2 / _right_link_2) as
  // a tree branch off the transmission hinge (revolute_2 / revolute_2_1) and
  // duplicating it as a visual-free helper body (..._loop_closure) hanging off
  // the slider link (revolute_3_loop_closure / _1). We deliberately leave this
  // topology untouched: the coupler's <visual> origin is authored in the
  // hinge-side frame, so re-parenting it onto the slider side would render the
  // mesh offset from the physical link. The loop is instead closed at runtime
  // with Drake constraints that recreate the slider-side revolute pivot between
  // transmission_link_1 and transmission_link_2. Every exported link and joint
  // name is preserved exactly.
  return ReadTextFileOrThrow(
      "models/robobee_assembly/urdf/robobee_assembly.urdf");
}

inline std::vector<drake::multibody::ModelInstanceIndex>
AddRoboBeeAssemblyModels(drake::multibody::Parser* parser) {
  return parser->AddModelsFromString(LoadRoboBeeAssemblyUrdfForDrake(), "urdf");
}

}  // namespace robobee_sim
