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

#include "pointcloudcrafter/pointcloudcrafter.hpp"

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
#include <vector>

#include "pointcloudcrafter/utils.hpp"
#include "pointcloudmodifyer.hpp"
namespace pointcloudcrafter
{
// global variables that will be populated by CLI arguments
std::string BAG_PATH;  // NOLINT
std::vector<std::string> TOPICS;
std::string OUT_DIR;            // NOLINT
std::string TARGET_FRAME = "";  // NOLINT
std::string TRANSFORM_FILE{};   // NOLINT
int64_t MAX_FRAMES = -1;
int64_t SKIP_FRAMES = 0;
int64_t STRIDE_FRAMES = 1;
bool SEQUENTIAL_NAMES = false;
bool BAG_TIME = false;
bool RELATIVE_TIME = false;
std::vector<double> CROPBOX{};
double CROPSPHERE{0.0};
double CROPCYLINDER{0.0};
std::vector<double> VOXELFILTER{};
std::pair<double, int> OUTLIERRADIUSFILTER{};
std::pair<double, int> OUTLIERSTATFILTER{};
bool PIE_FILTER = false;
/**
 * @brief PointCloudCrafter class
 */
PointCloudCrafter::PointCloudCrafter()
: reader_(BAG_PATH),
  tf2_buffer_(std::make_shared<rclcpp::Clock>()),
  logger_(rclcpp::get_logger("rosbag_to_pcd")),
  num_sensors_(TOPICS.size())
{
  // Check for number of topics
  if (TOPICS.size() > 4) {
    throw std::runtime_error("Only a maximum of 4 topics are supported");
  }
  // Initialize message to filter and sync to reader
  for (size_t i = 0; i < 4; i++) {
    std::string topic = (i < num_sensors_) ? TOPICS[i] : TOPICS[0];
    subscribers_.push_back(
      std::make_unique<tools::MsgFilter<sensor_msgs::msg::PointCloud2>>(topic, reader_, BAG_TIME));
  }
  // Initialize synchronizer for pointclouds
  synchronizer_ = std::make_unique<message_filters::Synchronizer<ApproxTimeSyncPolicy>>(
    ApproxTimeSyncPolicy(20), *subscribers_[0], *subscribers_[1], *subscribers_[2],
    *subscribers_[3]);
  // Register callback for synchronized pointclouds
  sync_connection_ = synchronizer_->registerCallback(std::bind(
    &PointCloudCrafter::pointcloud_sync_callback, this, std::placeholders::_1,
    std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));

  // Initialize transform listener
  reader_.add_listener<tf2_msgs::msg::TFMessage>(
    "/tf_static", std::bind(&PointCloudCrafter::tf_callback, this, std::placeholders::_1));
  tf2_buffer_.setUsingDedicatedThread(true);

  // Load transforms from file
  if (!TRANSFORM_FILE.empty()) {
    this->file_transforms_ = tools::utils::load_transforms_from_file(TRANSFORM_FILE);
  }

  // Init out directory
  if (OUT_DIR.empty()) {
    throw std::runtime_error("Output directory not specified");
  }
  if (!std::filesystem::exists(OUT_DIR)) {
    std::filesystem::create_directories(OUT_DIR);
  }
}
void PointCloudCrafter::run() { reader_.process(); }
void PointCloudCrafter::tf_callback(const tools::RosbagReaderMsg<tf2_msgs::msg::TFMessage> & msg)
{
  for (auto & tf : msg.ros_msg.transforms) {
    if (tf.header.frame_id == tf.child_frame_id) {
      continue;
    }
    tf2_buffer_.setTransform(tf, "bag", msg.bag_msg.topic_name == "/tf_static");
  }
}
void PointCloudCrafter::pointcloud_sync_callback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pc1,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pc2,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pc3,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pc4)
{
  // Check if we should skip frames
  if (SKIP_FRAMES > 0) {
    SKIP_FRAMES--;
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
void PointCloudCrafter::process_pointclouds(
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

    // adjust time information to have correct offset to base time
    uint64_t time_offset = tools::utils::timestamp_from_ros(msg.header.stamp) - base_time;
    try {
      if (RELATIVE_TIME) {
        // Point timestamps are relative to the header timestamp of each message
        tools::utils::set_timestamps(
          msg_transformed, tools::utils::timestamp_from_ros(msg.header.stamp), time_offset);
      } else {
        // Point timestamps are absolute timestamps
        tools::utils::set_timestamps(msg_transformed, 0, time_offset);
      }
    } catch (std::runtime_error &) {
      // do nothing if cloud has no field t
    }

    // Convert the pointcloud to PCL format and concatenate
    pcl::PCLPointCloud2 pc;
    pcl_conversions::toPCL(msg_transformed, pc);
    *merged_pc += pc;
  }
  stride_frames_ = stride_frames_ - 1;

  loaded_frames_++;
  if (MAX_FRAMES > 0 && loaded_frames_ >= MAX_FRAMES) {
    reader_.set_state(false);
  }

  // Modify the merged pointcloud with pointcloudmodifier
  // Pointcloud modifyer
  pointcloudmodifyer::Modifyer modifier;
  modifier.setCloud(merged_pc);
  // Apply filters
  // Cropbox filtering
  if (!CROPBOX.empty()) {
    modifier.cropBox(CROPBOX);
  }
  // Sphere filtering
  if (CROPSPHERE > 0.0) {
    modifier.cropSphere(CROPSPHERE);
  }
  // Cylinder filtering
  if (CROPCYLINDER > 0.0) {
    modifier.cropCylinder(CROPCYLINDER);
  }
  // Voxelization
  if (!VOXELFILTER.empty()) {
    modifier.voxelFilter(VOXELFILTER);
  }
  // Outlier radius filtering
  if (!OUTLIERRADIUSFILTER.first > 0.0) {
    modifier.outlierRadiusFilter(OUTLIERRADIUSFILTER.first, OUTLIERRADIUSFILTER.second);
  }
  // Outlier statistical filtering
  if (!OUTLIERSTATFILTER.first > 0.0) {
    modifier.outlierStatFilter(OUTLIERSTATFILTER.first, OUTLIERSTATFILTER.second);
  }

  // Save output cloud
  auto ts = tools::utils::timestamp_to_ros(base_time);
  std::string name = fmt::format("{}_{:09}", ts.sec, ts.nanosec);
  if (!modifier.savePCD(OUT_DIR + "/" + name + ".pcd")) {
    RCLCPP_ERROR(logger_, "Failed to save pointcloud to %s", name.c_str());
    return;
  }
}
void PointCloudCrafter::transform_pc(
  const sensor_msgs::msg::PointCloud2 & msg_in, sensor_msgs::msg::PointCloud2 & msg_out)
{
  Eigen::Affine3d transformation(Eigen::Affine3d::Identity());

  if (file_transforms_.find(msg_in.header.frame_id) != file_transforms_.end()) {
    transformation = file_transforms_[msg_in.header.frame_id];
  } else if (!TARGET_FRAME.empty()) {
    transformation = tools::utils::transform2eigen(
      tf2_buffer_.lookupTransform(TARGET_FRAME, msg_in.header.frame_id, rclcpp::Time{0}));
  }

  tools::utils::transform_pointcloud2(transformation.cast<float>(), msg_in, msg_out);
}
}  // namespace pointcloudcrafter
