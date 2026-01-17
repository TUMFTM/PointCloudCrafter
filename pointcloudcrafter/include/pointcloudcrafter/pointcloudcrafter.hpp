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
#include <message_filters/connection.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <tf2_ros/buffer.h>

#include <Eigen/Eigen>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <tf2_msgs/msg/detail/tf_message__struct.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <unordered_map>
#include <utility>
#include <vector>

#include "pointcloudcrafter/rosbag_reader.hpp"
#include "cli_config.hpp"

// clang-format off
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2> ApproxTimeSyncPolicy;  // NOLINT
// clang-format on
namespace pointcloudcrafter
{
/**
 * @brief PointCloudCrafter class
 */
class PointCloudCrafter
{
public:
  /**
   * @brief Constructor
   */
  explicit PointCloudCrafter(const config::CrafterConfig & cfg);
  /**
   * @brief Run the pointcloud crafter
   */
  void run() { reader_.process(); }

protected:
  /**
   * @brief Callback for TF messages
   * @param msg - TF message
   */
  void tf_callback(const tools::RosbagReaderMsg<tf2_msgs::msg::TFMessage> & msg);
  /**
   * @brief Callback for synchronized pointclouds
   * @param pc1 - first pointcloud
   * @param pc2 - second pointcloud
   * @param pc3 - third pointcloud
   * @param pc4 - fourth pointcloud
   */
  void pointcloud_sync_callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pc1,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pc2,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pc3,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pc4);
  /**
   * @brief Process the pointclouds once all pointclouds are synchronized
   * @brief -> to be called after all pointclouds are synchronized
   * @brief -> apply filters and save the pointcloud
   * @param pc_msgs - vector of pointclouds
   */
  void process_pointclouds(std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr> & pc_msgs);
  /**
   * @brief Transform a pointcloud given the transform from either the file or the TF buffer
   * @param msg_in - input pointcloud
   * @param msg_out - output pointcloud
   */
  void transform_pc(
    const sensor_msgs::msg::PointCloud2 & msg_in, sensor_msgs::msg::PointCloud2 & msg_out);

private:
  // CLI Configuration
  config::CrafterConfig cfg_;
  // Custom rosbag reader
  tools::RosbagReader reader_;
  // TF2 buffer to store transform
  tf2_ros::Buffer tf2_buffer_;
  // ROS logger
  rclcpp::Logger logger_;
  // Subscribers for pointclouds
  std::vector<std::unique_ptr<tools::MsgFilter<sensor_msgs::msg::PointCloud2>>> subscribers_;
  // Synchronizer for pointclouds
  std::unique_ptr<message_filters::Synchronizer<ApproxTimeSyncPolicy>> synchronizer_;
  message_filters::Connection sync_connection_{};
  // Map to store transforms given by file
  std::unordered_map<std::string, Eigen::Affine3d> file_transforms_{};

  // Storage
  size_t num_sensors_{};
  int64_t loaded_frames_{0};
  int64_t stride_frames_{0};
  int64_t skip_frames_{0};
};
}  // namespace pointcloudcrafter
