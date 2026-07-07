// One-shot diagnostic: prints the world-frame span/chord/normal axes of both
// wings in the default (t=0) configuration and checks whether the right wing is
// the sagittal-plane mirror of the left. This answers whether the two wing
// body frames are set up as a true mirror pair or merely as rotated copies,
// which determines whether the residual roll/yaw torque comes from the aero
// frame setup or from somewhere else.
//
// It reuses the same axis construction as visualize_robobee_aeromechanical.cc
// and welds the root to world exactly like the fixed build, so the frames match
// what the running simulator sees.

#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>

#include <Eigen/Dense>

#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"

#include "aeromechanical_wing_constants.h"

namespace {

constexpr char kPackageName[] = "robobee_assembly";
constexpr char kPackagePath[] = "models/robobee_assembly";
constexpr char kAssemblyUrl[] =
    "package://robobee_assembly/urdf/robobee_assembly.urdf";

// Identical to CalcAeroSpanAxisInBody / CalcAeroChordAxisInBody in
// visualize_robobee_aeromechanical.cc.
Eigen::Vector3d SpanAxisInBody(
    const robobee::AeromechanicalWingConstants<400>& c) {
  const double cs = std::cos(c.span_axis_angle_rad);
  const double sn = std::sin(c.span_axis_angle_rad);
  return (cs * Eigen::Vector3d::UnitX() + sn * Eigen::Vector3d::UnitY())
      .normalized();
}

Eigen::Vector3d ChordAxisInBody(
    const robobee::AeromechanicalWingConstants<400>& c) {
  const double cs = std::cos(c.span_axis_angle_rad);
  const double sn = std::sin(c.span_axis_angle_rad);
  const double chord_sign = c.chord_axis_sign >= 0.0 ? 1.0 : -1.0;
  return (chord_sign *
          (-sn * Eigen::Vector3d::UnitX() + cs * Eigen::Vector3d::UnitY()))
      .normalized();
}

// A representative spanwise point in body frame: the tip station mid-chord,
// built the same way as p_BoP_B in the aero model.
Eigen::Vector3d TipMidChordInBody(
    const robobee::AeromechanicalWingConstants<400>& c,
    const Eigen::Vector3d& span_axis_B, const Eigen::Vector3d& chord_axis_B) {
  const auto& tip = c.blade_stations.back();
  const double x_r_m = c.x_r_hat_by_R * c.span_R_m;
  return (tip.r_m + x_r_m) * span_axis_B +
         tip.y_hinge_to_mid_chord_hat * c.mean_chord_cbar_m * chord_axis_B;
}

void PrintVec(const std::string& label, const Eigen::Vector3d& v) {
  std::cout << std::setw(22) << std::left << label << std::right
            << std::setprecision(9) << std::fixed << std::setw(15) << v.x()
            << std::setw(15) << v.y() << std::setw(15) << v.z() << "\n";
}

}  // namespace

int main() {
  drake::multibody::MultibodyPlant<double> plant(0.0);
  drake::multibody::Parser parser(&plant);
  parser.package_map().Add(kPackageName, kPackagePath);
  parser.AddModelsFromUrl(kAssemblyUrl);
  // Match the fixed build: weld the root to world (identity), so world == root.
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("root"));
  plant.Finalize();

  std::unique_ptr<drake::systems::Context<double>> context =
      plant.CreateDefaultContext();

  const drake::math::RigidTransformd X_WR =
      plant.GetBodyByName("root").EvalPoseInWorld(*context);
  std::cout << "X_WR translation: " << X_WR.translation().transpose()
            << "  (should be ~0; world == root)\n\n";

  struct WingRef {
    std::string label;
    std::string body_name;
    const robobee::AeromechanicalWingConstants<400>* constants;
  };
  const WingRef left{"left", "wing_membrane",
                     &robobee::kLeftWingAeromechanicalConstants};
  const WingRef right{"right", "wing_membrane_1",
                      &robobee::kRightWingAeromechanicalConstants};

  Eigen::Vector3d span_W[2], chord_W[2], normal_W[2], tip_W[2], origin_W[2];
  int idx = 0;
  for (const WingRef* w : {&left, &right}) {
    const Eigen::Vector3d span_B = SpanAxisInBody(*w->constants);
    const Eigen::Vector3d chord_B = ChordAxisInBody(*w->constants);
    const Eigen::Vector3d normal_B = Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d tip_B =
        TipMidChordInBody(*w->constants, span_B, chord_B);

    const drake::math::RigidTransformd X_WB =
        plant.GetBodyByName(w->body_name).EvalPoseInWorld(*context);
    const Eigen::Matrix3d R_WB = X_WB.rotation().matrix();

    span_W[idx] = (R_WB * span_B).normalized();
    chord_W[idx] = (R_WB * chord_B).normalized();
    normal_W[idx] = (R_WB * normal_B).normalized();
    tip_W[idx] = X_WB * tip_B;
    origin_W[idx] = X_WB.translation();

    std::cout << "=== " << w->label << " wing (" << w->body_name << ") ===\n";
    std::cout << "                            x              y              z\n";
    PrintVec("span_W", span_W[idx]);
    PrintVec("chord_W", chord_W[idx]);
    PrintVec("normal_W", normal_W[idx]);
    PrintVec("body_origin_W", origin_W[idx]);
    PrintVec("tip_midchord_W", tip_W[idx]);
    std::cout << "\n";
    ++idx;
  }

  // Sagittal-plane mirror: the controller-convention lateral (left/right) axis
  // is the root x-axis (root +y is forward, root -x is "left"), and root ==
  // world here. So the sagittal plane is the world y-z plane and the mirror
  // reflects the world x-component.
  const Eigen::Matrix3d M = Eigen::Vector3d(-1.0, 1.0, 1.0).asDiagonal();

  auto report = [](const std::string& label, const Eigen::Vector3d& mirror_right,
                   const Eigen::Vector3d& left_vec) {
    const double res_plus = (mirror_right - left_vec).norm();
    const double res_minus = (mirror_right + left_vec).norm();
    const bool flip = res_minus < res_plus;
    std::cout << std::setw(18) << std::left << label << std::right
              << "  residual=" << std::setprecision(3) << std::scientific
              << std::min(res_plus, res_minus)
              << (flip ? "  (right maps to -left)" : "  (right maps to +left)")
              << "\n";
  };

  std::cout << "=== mirror check: M*right  vs  left  (M flips world x) ===\n";
  std::cout << "A true mirror pair => small residuals below (< ~1e-3).\n";
  report("span", M * span_W[1], span_W[0]);
  report("chord", M * chord_W[1], chord_W[0]);
  report("normal", M * normal_W[1], normal_W[0]);
  report("tip_midchord", M * tip_W[1], tip_W[0]);
  report("body_origin", M * origin_W[1], origin_W[0]);

  return 0;
}
