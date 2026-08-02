#include "pclCalcTransform.hpp"

#include <Eigen/Geometry>
#include <cmath>
#include <iostream>

int main()
{
  YsPCLCalcTransform solver;
  solver.target_box_min = Eigen::Vector4f(0.3f, -0.4f, 0.3f, 1.0f);
  solver.target_box_max = Eigen::Vector4f(1.1f, 0.4f, 1.0f, 1.0f);
  solver.curve_box_min = Eigen::Vector4f(-0.25f, -0.20f, -0.01f, 1.0f);
  solver.curve_box_max = Eigen::Vector4f(0.25f, 0.20f, 0.01f, 1.0f);
  solver.plane_fit_enabled = true;
  solver.plane_fit_distance_threshold = 0.002;
  solver.plane_fit_max_iterations = 1000;
  solver.plane_fit_min_inliers = 1000;
  solver.plane_fit_min_inlier_ratio = 0.5;
  solver.plane_fit_max_rms = 0.0015;
  solver.plane_fit_max_normal_angle_deg = 10.0;
  solver.tool_align_world = false;

  Eigen::Vector3f expected = solver.expectedWorldX();
  Eigen::Vector3f world_up = solver.world_up_in_base.normalized();
  Eigen::Vector3f nominal_y = world_up - world_up.dot(expected) * expected;
  nominal_y.normalize();
  Eigen::Vector3f nominal_x = nominal_y.cross(expected).normalized();

  constexpr float pi = 3.14159265358979323846f;
  const Eigen::Vector3f desired_normal =
    Eigen::AngleAxisf(3.0f * pi / 180.0f, nominal_x) * expected;
  Eigen::Vector3f desired_y = world_up - world_up.dot(desired_normal) * desired_normal;
  desired_y.normalize();
  const Eigen::Vector3f desired_x = desired_y.cross(desired_normal).normalized();
  const Eigen::Vector3f center(0.68f, -0.01f, 0.66f);

  auto cloud = std::make_shared<YsPCLCalcTransform::PointCloud>();
  cloud->reserve(25000);
  for (int iu = -100; iu <= 100; ++iu) {
    for (int iv = -60; iv <= 60; ++iv) {
      const float u = static_cast<float>(iu) * 0.0015f;
      const float v = static_cast<float>(iv) * 0.0015f;
      const float noise = 0.0004f * std::sin(0.37f * iu + 0.19f * iv);
      const Eigen::Vector3f point = center + u * desired_x + v * desired_y
        + noise * desired_normal;
      cloud->push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
    }
  }
  // 确定性的离群点，验证拟合不会被非平面点拉偏。
  for (int i = 0; i < 500; ++i) {
    const float a = static_cast<float>((i * 37) % 101) / 100.0f;
    const float b = static_cast<float>((i * 53) % 97) / 96.0f;
    const float c = static_cast<float>((i * 71) % 89) / 88.0f;
    cloud->push_back(pcl::PointXYZ(
      0.45f + 0.45f * a, -0.25f + 0.50f * b, 0.45f + 0.40f * c));
  }
  cloud->width = static_cast<std::uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = true;

  solver.plane_ox = center.x();
  solver.plane_oy = center.y();
  Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
  if (!solver.calc(cloud, result)) {
    std::cerr << "synthetic plane fit unexpectedly failed" << std::endl;
    return 1;
  }

  const Eigen::Vector3f fitted_normal = result.block<3, 1>(0, 2).normalized();
  const float dot = fitted_normal.dot(desired_normal);
  const float angle_error_deg =
    std::acos(std::max(-1.0f, std::min(1.0f, dot))) * 180.0f / pi;
  std::cout << "synthetic desired normal=" << desired_normal.transpose()
            << " fitted=" << fitted_normal.transpose()
            << " angle_error=" << angle_error_deg << "deg" << std::endl;
  if (dot < std::cos(0.3f * pi / 180.0f)) {
    std::cerr << "fitted tool Z does not match the synthetic plane normal" << std::endl;
    return 2;
  }
  if (fitted_normal.dot(expected) <= 0.0f) {
    std::cerr << "fitted normal sign flipped away from world X+" << std::endl;
    return 3;
  }
  return 0;
}
