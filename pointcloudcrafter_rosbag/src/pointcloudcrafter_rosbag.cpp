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

#include "pointcloudcrafter_rosbag/pointcloudcrafter_rosbag.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/util/Constants.h>
#include <fmt/core.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>
#include <pcl/make_shared.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>

#include <Eigen/Eigen>
#include <filesystem>  // NOLINT
#include <functional>
#include <limits>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "utils.hpp"
#include "pointcloudmodifier.hpp"
namespace pointcloudcrafter
{
/**
 * @brief Rosbag class
 */
Rosbag::Rosbag(const config::RosbagConfig & cfg)
: reader_(cfg_.bag_path),
  tf2_buffer_(std::make_shared<rclcpp::Clock>()),
  logger_(rclcpp::get_logger("pointcloudcrafter_rosbag")),
  num_sensors_(cfg_.topics.size()),
  cfg_(cfg)
{
  // Check for number of topics
  if (cfg_.topics.size() > 4) {
    throw std::runtime_error("Only a maximum of 4 topics are supported");
  }
  // Initialize message to filter and sync to reader
  for (size_t i = 0; i < 4; i++) {
    std::string topic = (i < num_sensors_) ? cfg_.topics[i] : cfg_.topics[0];
    subscribers_.push_back(
      std::make_unique<tools::MsgFilter<sensor_msgs::msg::PointCloud2>>(topic, reader_));
  }
  // Initialize synchronizer for pointclouds
  synchronizer_ = std::make_unique<message_filters::Synchronizer<ApproxTimeSyncPolicy>>(
    ApproxTimeSyncPolicy(20), *subscribers_[0], *subscribers_[1], *subscribers_[2],
    *subscribers_[3]);
  // Register callback for synchronized pointclouds
  sync_connection_ = synchronizer_->registerCallback(std::bind(
    &Rosbag::pointcloud_sync_callback, this, std::placeholders::_1,
    std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));

  // Initialize transform listener
  reader_.add_listener<tf2_msgs::msg::TFMessage>(
    "/tf_static", std::bind(&Rosbag::tf_callback, this, std::placeholders::_1));
  tf2_buffer_.setUsingDedicatedThread(true);

  // Load transforms from file
  if (!cfg_.transform_file.empty()) {
    this->file_transforms_ = tools::utils::load_transforms_from_file(cfg_.transform_file);
  }

  // Init out directory
  // TODO(ga58lar): remove this check as CLI should ensure this
  if (cfg_.out_dir.empty()) {
    throw std::runtime_error("Output directory not specified");
  }
  if (!std::filesystem::exists(cfg_.out_dir)) {
    std::filesystem::create_directories(cfg_.out_dir);
  }
}
void Rosbag::tf_callback(const tools::RosbagReaderMsg<tf2_msgs::msg::TFMessage> & msg)
{
  for (auto & tf : msg.ros_msg.transforms) {
    if (tf.header.frame_id == tf.child_frame_id) {
      continue;
    }
    tf2_buffer_.setTransform(tf, "bag", msg.bag_msg.topic_name == "/tf_static");
  }
}
void Rosbag::pointcloud_sync_callback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pc1,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pc2,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pc3,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pc4)
{
  // Check if we should skip frames
  if (cfg_.skip_frames > 0) {
    skip_frames_--;
    return;
  }

  if (stride_frames_ > 0) {
    stride_frames_--;
    return;
  }

  // Put all pointclouds into a vector
  // Attention: size of vector may not always equal number of sensors!
  // -> Resize to number of sensors
  std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr> pc_msgs{pc1, pc2, pc3, pc4};
  pc_msgs.resize(num_sensors_);

  // Process the pointclouds
  process_pointclouds(pc_msgs);
}
void Rosbag::process_pointclouds(
  std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr> & pc_msgs)
{
  // Find the smallest timestamp of all pointclouds
  uint64_t base_time = std::numeric_limits<uint64_t>::max();
  // find the smallest timestamp
  for (auto & msg : pc_msgs) {
    uint64_t time = tools::utils::timestamp_from_ros(msg->header.stamp);
    if (time < base_time) {
      base_time = time;
    }
  }

  // Concatenate all pointclouds into one
  auto merged_pc = pcl::make_shared<pcl::PCLPointCloud2>();
  for (size_t i = 0; i < pc_msgs.size(); i++) {
    // Transform the pointcloud to the target frame
    const sensor_msgs::msg::PointCloud2 & msg = *pc_msgs[i];
    sensor_msgs::msg::PointCloud2 msg_transformed;
    transform_pc(msg, msg_transformed);

    // Convert the pointcloud to PCL format and concatenate
    pcl::PCLPointCloud2 pc;
    pcl_conversions::toPCL(msg_transformed, pc);
    *merged_pc += pc;
  }
  stride_frames_ = cfg_.stride_frames - 1;

  loaded_frames_++;
  if (cfg_.max_frames > 0 && loaded_frames_ >= cfg_.max_frames) {
    reader_.set_state(false);
  }

  // Modify the merged pointcloud with pointcloudmodifierlib
  // Pointcloud modifier
  pointcloudmodifierlib::Modifier modifier;
  modifier.setCloud(merged_pc);
  // Apply filters
  // Cropbox filtering
  if (!cfg_.cropbox.empty()) {
    modifier.cropBox(cfg_.cropbox, cfg_.inverse_crop);
  }
  // Sphere filtering
  if (cfg_.cropsphere > 0.0) {
    modifier.cropSphere(cfg_.cropsphere, cfg_.inverse_crop);
  }
  // Cylinder filtering
  if (cfg_.cropcylinder > 0.0) {
    modifier.cropCylinder(cfg_.cropcylinder, cfg_.inverse_crop);
  }
  // Voxelization
  if (!cfg_.voxelfilter.empty()) {
    modifier.voxelFilter(cfg_.voxelfilter);
  }
  // Outlier radius filtering
  if (cfg_.outlier_radius_filter.first > 0.0) {
    modifier.outlierRadiusFilter(cfg_.outlier_radius_filter.first,
      cfg_.outlier_radius_filter.second);
  }
  // Outlier statistical filtering
  if (cfg_.outlier_stat_filter.first > 0.0) {
    modifier.outlierStatFilter(cfg_.outlier_stat_filter.first, cfg_.outlier_stat_filter.second);
  }

  // Save output cloud
  auto ts = tools::utils::timestamp_to_ros(base_time);
  std::string name = fmt::format("{}_{:09}", ts.sec, ts.nanosec);
  if (!modifier.savePCD(cfg_.out_dir + "/" + name + ".pcd")) {
    RCLCPP_ERROR(logger_, "Failed to save pointcloud to %s", name.c_str());
    return;
  }

  // Save timestamps if enabled
  if (cfg_.timestamps) {
    modifier.timestampAnalyzer(cfg_.out_dir + "/" + name + "_stamps.txt");
  }
}
void Rosbag::transform_pc(
  const sensor_msgs::msg::PointCloud2 & msg_in, sensor_msgs::msg::PointCloud2 & msg_out)
{
  Eigen::Affine3d transformation(Eigen::Affine3d::Identity());

  if (file_transforms_.find(msg_in.header.frame_id) != file_transforms_.end()) {
    transformation = file_transforms_[msg_in.header.frame_id];
  } else if (!cfg_.target_frame.empty()) {
    transformation = tools::utils::transform2eigen(
      tf2_buffer_.lookupTransform(cfg_.target_frame, msg_in.header.frame_id, rclcpp::Time{0}));
  }

  tools::utils::transform_pointcloud2(transformation.cast<float>(), msg_in, msg_out);
}
}  // namespace pointcloudcrafter
