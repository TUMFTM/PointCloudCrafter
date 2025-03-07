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
#include <cinttypes>
#include <cstdint>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <string>
#include <vector>
#include <pointcloudcrafter/point_types.hpp>
namespace pointcloudcrafter::tools::utils
{

void save_timestamps(const std::string & filename, const std::vector<uint64_t> & timestamps);

uint64_t timestamp_from_ros(const builtin_interfaces::msg::Time & ros);

builtin_interfaces::msg::Time timestamp_to_ros(uint64_t stamp);

void transform_pointcloud2(
  const Eigen::Affine3f & transform, const sensor_msgs::msg::PointCloud2 & pc_in,
  sensor_msgs::msg::PointCloud2 & pc_out);

}  // namespace pointcloudcrafter::tools::utils
