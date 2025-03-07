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
extern std::vector<float> GEOMETRIC_FILTERING;
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
  std::vector<uint64_t> timestamps_lidar_{};
  int64_t loaded_frames_{0};
  int64_t stride_frames_{0};
};
}  // namespace pointcloudcrafter
