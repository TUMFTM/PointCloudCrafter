// Copyright 2024 Maximilian Leitenstern

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <string>
#include <vector>

#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

int analyze_timestamps(std::string input_path, std::string topic_name);

/**
 * @brief extract timestamps from pointcloud message
 *
 * @param[in] msg                - sensor_msgs::msg::PointCloud2:
 *                                 pointcloud message
 * @param[out]                   - std::vector<double>:
 *                                 timestamps of single points
 */
std::vector<double> GetTimestamps(const sensor_msgs::msg::PointCloud2 & msg);

/**
 * @brief extract timestamp fields from pointcloud2 message
 *
 * @param[in] msg                - sensor_msgs::msg::PointCloud2:
 *                                 pointcloud message
 * @param[out]                   - sensor_msgs::msg::PointField
 *                                 output field
 */
sensor_msgs::msg::PointField GetTimestampField(const sensor_msgs::msg::PointCloud2 & msg);

/**
 * @brief extract timestamps from pointcloud message
 *
 * @param[in] msg                - sensor_msgs::msg::PointCloud2:
 *                                 input pointcloud message
 * @param[in] field              - sensor_msgs::msg::PointField:
 *                                 field containing the timestamps
 * @param[out]                   - std::vector<double>:
 *                                 timestamps
 */
std::vector<double> ExtractTimestampsFromMsg(
  const sensor_msgs::msg::PointCloud2 & msg, const sensor_msgs::msg::PointField & field);
