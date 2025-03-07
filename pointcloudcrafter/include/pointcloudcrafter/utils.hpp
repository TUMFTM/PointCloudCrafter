/*
 * Copyright (C) 2024 Markus Pielmeier, Florian Sauerbeck, Dominik Kulmer, Maximilian Leitenstern
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <Eigen/Eigen>
#include <builtin_interfaces/msg/time.hpp>
#include <fstream>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <string>
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
/**
 * @brief extract timestamps from pointcloud message
 * @param [in]          sensor_msgs::msg::PointCloud2
 *                      input pointcloud message
 * @param [out]         std::vector<std::uint64_t>
 *                      timestamps
 */
std::vector<std::uint64_t> extract_timestamps(const sensor_msgs::msg::PointCloud2 & msg)
{
  sensor_msgs::msg::PointField timestamp_field;
  for (const auto & field : msg.fields) {
    if (field.name == "timestamp" || field.name == "time_stamp" || field.name == "t") {
      timestamp_field = field;
    }
  }
  if (!timestamp_field.count) {
    throw std::runtime_error("No field with timestamps found!");
  }
  // Extract timestamps from cloud_msg
  const size_t n_points = msg.height * msg.width;
  std::vector<std::uint64_t> timestamps;
  timestamps.reserve(n_points);

  // Timestamps are doubles -> time in sec as offset to center time
  if (
    timestamp_field.name == "timestamp" || timestamp_field.name == "time_stamp" ||
    timestamp_field.name == "t") {
    if (timestamp_field.datatype == pcl::PCLPointField::FLOAT32) {  // type 7
      sensor_msgs::PointCloud2ConstIterator<float> msg_t(msg, timestamp_field.name);
      for (size_t i = 0; i < n_points; ++i, ++msg_t) {
        timestamps.emplace_back(static_cast<std::uint64_t>(*msg_t * BILLION));
      }
    } else if (timestamp_field.datatype == pcl::PCLPointField::FLOAT64) {  // type 8
      sensor_msgs::PointCloud2ConstIterator<double> msg_t(msg, timestamp_field.name);
      for (size_t i = 0; i < n_points; ++i, ++msg_t) {
        timestamps.emplace_back(static_cast<std::uint64_t>(*msg_t * BILLION));
      }
    } else if (timestamp_field.datatype == pcl::PCLPointField::UINT32) {  // type 6
      sensor_msgs::PointCloud2ConstIterator<uint32_t> msg_t(msg, timestamp_field.name);
      for (size_t i = 0; i < n_points; ++i, ++msg_t) {
        timestamps.emplace_back(static_cast<std::uint64_t>(*msg_t * BILLION));
      }
    } else if (timestamp_field.datatype == pcl::PCLPointField::UINT8) {  // type 2 - array of 8
                                                                         // uint8
      sensor_msgs::PointCloud2ConstIterator<uint8_t> msg_t(msg, timestamp_field.name);
      for (size_t i = 0; i < n_points; ++i, ++msg_t) {
        uint64_t stamp;
        std::memcpy(&stamp, &*msg_t, sizeof(uint64_t));
        timestamps.emplace_back(static_cast<double>(stamp));
      }
    } else {
      std::cout << "Time field of type != 2,6,7,8" << std::endl;
      exit(EXIT_FAILURE);
    }
  }
  return timestamps;
}
    //   // Print properties of current msgs
    //   std::cout << "PointCloud" << std::endl;
    //   std::cout << "size: " << stamps.size() << std::endl;
    //   std::cout << std::fixed << std::setprecision(9);
    //   std::cout << "msg_stamp: " << msg_stamp * (1e-9) << std::endl;
    //   std::cout << "max_stamp: " << *max_element(stamps.begin(), stamps.end()) * (1e-9) << std::endl
    //             << "min_stamp: " << *min_element(stamps.begin(), stamps.end()) * (1e-9)
    //             << std::endl;
    //   bag_diff.push_back(diff);
}  // namespace pointcloudcrafter::tools::utils
