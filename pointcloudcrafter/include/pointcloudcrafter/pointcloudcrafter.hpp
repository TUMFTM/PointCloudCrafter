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
#include <vector>
#include <utility>

#include "pointcloudcrafter/rosbag_reader.hpp"

// clang-format off
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2> ApproxTimeSyncPolicy;  // NOLINT
// clang-format on
namespace pointcloudcrafter
{
// Flags
extern std::string BAG_PATH;
extern std::vector<std::string> TOPICS;
extern std::string OUT_DIR;
extern std::string TARGET_FRAME;
extern std::string SENSOR_NUMBER_FIELD;
extern std::string TRANSFORM_FILE;
extern int64_t MAX_FRAMES;
extern int64_t SKIP_FRAMES;
extern int64_t STRIDE_FRAMES;
extern bool SEQUENTIAL_NAMES;
extern bool BAG_TIME;
extern bool RELATIVE_TIME;
extern std::vector<double> CROPBOX;
extern double CROPSPHERE;
extern double CROPCYLINDER;
extern std::vector<double> VOXELFILTER;
extern std::pair<double, int> OUTLIERRADIUSFILTER;
extern std::pair<double, int> OUTLIERSTATFILTER;
extern bool PIE_FILTER;
/**
 * @brief PointCloudCrafter class
 */
class PointCloudCrafter
{
public:
  PointCloudCrafter();
  void run();

protected:
  void tf_callback(const tools::RosbagReaderMsg<tf2_msgs::msg::TFMessage> & msg);

  void pointcloud_sync_callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pc1,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pc2,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pc3,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pc4);

  void process_pointclouds(std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr> & pc_msgs);

  void transform_pc(
    const sensor_msgs::msg::PointCloud2 & msg_in, sensor_msgs::msg::PointCloud2 & msg_out);

private:
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
  std::unordered_map<std::string, Eigen::Affine3d> file_transforms_;

  // Storage
  size_t num_sensors_{};
  int64_t loaded_frames_{0};
  int64_t stride_frames_{0};
};
}  // namespace pointcloudcrafter
