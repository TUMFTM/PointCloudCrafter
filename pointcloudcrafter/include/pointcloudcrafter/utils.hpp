/*
 * Copyright 2024 2024 Markus Pielmeier, Florian Sauerbeck,
 * Dominik Kulmer, Maximilian Leitenstern
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <Eigen/Eigen>
#include <builtin_interfaces/msg/time.hpp>
#include <fstream>
#include <iostream>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <string>
#include <unordered_map>
#include <vector>
namespace pointcloudcrafter::tools::utils
{
constexpr std::uint64_t BILLION = 1000000000;
constexpr float COLORS[] = {0.0, 1.0, 0.333, 0.666};
/**
 * @brief Convert a ROS timestamp to a uint64_t
 * @param ros - ROS timestamp
 * @return uint64_t timestamp in nanoseconds
 */
uint64_t timestamp_from_ros(const builtin_interfaces::msg::Time & ros)
{
  return static_cast<std::uint64_t>(ros.sec) * BILLION + static_cast<std::uint64_t>(ros.nanosec);
}
/**
 * @brief Convert a uint64_t timestamp to a ROS timestamp
 * @param stamp - uint64_t timestamp in nanoseconds
 * @return ROS timestamp
 */
builtin_interfaces::msg::Time timestamp_to_ros(std::uint64_t stamp)
{
  builtin_interfaces::msg::Time ros;
  ros.sec = static_cast<std::int32_t>(stamp / BILLION);
  ros.nanosec = static_cast<std::uint32_t>(stamp % BILLION);
  return ros;
}
/**
 * @brief Convert a ROS transform message to an Eigen transform
 * @param transform - ROS transform message
 * @return Eigen transform
 */
Eigen::Affine3d transform2eigen(const geometry_msgs::msg::TransformStamped & transform)
{
  Eigen::Affine3d affine = Eigen::Affine3d::Identity();
  const auto & t = transform.transform;
  affine.translate(Eigen::Vector3d{t.translation.x, t.translation.y, t.translation.z});
  affine.rotate(Eigen::Quaterniond{t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z});
  return affine;
}
/**
 * @brief Transform a pointcloud2 message using an Eigen transform
 * @param transform - Eigen transform
 * @param pc_in - input pointcloud2 message
 * @param pc_out - output pointcloud2 message
 */
void transform_pointcloud2(
  const Eigen::Affine3f & transform, const sensor_msgs::msg::PointCloud2 & pc_in,
  sensor_msgs::msg::PointCloud2 & pc_out)
{
  // Make a copy first so that the headers are staying the same
  pc_out = pc_in;

  sensor_msgs::PointCloud2ConstIterator<float> it_x_in(pc_in, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y_in(pc_in, "y");
  sensor_msgs::PointCloud2ConstIterator<float> it_z_in(pc_in, "z");
  sensor_msgs::PointCloud2Iterator<float> it_x_out(pc_out, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y_out(pc_out, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z_out(pc_out, "z");

  for (; it_x_in != it_x_in.end();
       ++it_x_in, ++it_y_in, ++it_z_in, ++it_x_out, ++it_y_out, ++it_z_out) {
    Eigen::Vector4f point_in{*it_x_in, *it_y_in, *it_z_in, 1.0f};
    Eigen::Vector4f point_out = transform * point_in;
    *it_x_out = point_out.x();
    *it_y_out = point_out.y();
    *it_z_out = point_out.z();
  }
}
std::unordered_map<std::string, Eigen::Affine3d> load_transforms_from_file(
  const std::string & filename)
{
  std::unordered_map<std::string, Eigen::Affine3d> transforms;

  std::ifstream file(filename);
  if (!file.is_open()) {
    throw std::runtime_error("Cannot open file: " + filename);
  }

  std::string frameId;
  double x, y, z;
  double r1, r2, r3, r4, r5, r6, r7, r8, r9;

  std::string line;
  while (std::getline(file, line)) {
    if (line.empty()) {
      continue;
    }
    std::istringstream iss(line);

    if (!(iss >> frameId >> r1 >> r2 >> r3 >> x >> r4 >> r5 >> r6 >> y >> r7 >> r8 >> r9 >> z)) {
      std::cerr << "Transform not in expected format" << std::endl;
    }
    Eigen::Matrix3d rotation;
    rotation << r1, r2, r3, r4, r5, r6, r7, r8, r9;

    Eigen::Affine3d trafo = Eigen::Affine3d::Identity();
    trafo.linear() = rotation;
    trafo.translation() << x, y, z;

    transforms[frameId] = trafo;
  }

  return transforms;
}
}  // namespace pointcloudcrafter::tools::utils
