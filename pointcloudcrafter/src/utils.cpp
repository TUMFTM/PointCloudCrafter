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

#include "pointcloudcrafter/utils.hpp"

#include <fmt/format.h>

#include <cstdint>
#include <fstream>
namespace pointcloudcrafter::tools::utils
{
constexpr auto one_billion = 1000000000L;
void save_timestamps(const std::string & filename, const std::vector<uint64_t> & timestamps)
{
  std::ofstream gps_file;
  gps_file.open(filename);
  for (uint64_t timestamp : timestamps) {
    gps_file << fmt::format("{} {:09d}\n", timestamp / one_billion, timestamp % one_billion);
  }
}
uint64_t timestamp_from_ros(const builtin_interfaces::msg::Time & ros)
{
  return static_cast<uint64_t>(ros.sec) * one_billion + static_cast<uint64_t>(ros.nanosec);
}
builtin_interfaces::msg::Time timestamp_to_ros(uint64_t stamp)
{
  builtin_interfaces::msg::Time ros;
  ros.sec = static_cast<int32_t>(stamp / one_billion);
  ros.nanosec = static_cast<int32_t>(stamp % one_billion);
  return ros;
}
void transform_pointcloud2(
  const Eigen::Affine3f & transform, const sensor_msgs::msg::PointCloud2 & pc_in,
  sensor_msgs::msg::PointCloud2 & pc_out)
{
  using ConstIt = sensor_msgs::PointCloud2ConstIterator<float>;
  using It = sensor_msgs::PointCloud2Iterator<float>;
  // Make a copy first so that the headers are correct
  pc_out = pc_in;

  ConstIt it_x_in(pc_in, "x");
  ConstIt it_y_in(pc_in, "y");
  ConstIt it_z_in(pc_in, "z");
  It it_x_out(pc_out, "x");
  It it_y_out(pc_out, "y");
  It it_z_out(pc_out, "z");

  for (; it_x_in != it_x_in.end();
       ++it_x_in, ++it_y_in, ++it_z_in, ++it_x_out, ++it_y_out, ++it_z_out) {
    Eigen::Vector4f point_in{*it_x_in, *it_y_in, *it_z_in, 1.0f};
    Eigen::Vector4f point_out = transform * point_in;
    *it_x_out = point_out.x();
    *it_y_out = point_out.y();
    *it_z_out = point_out.z();
  }
}
}  // namespace pointcloudcrafter::tools::utils
