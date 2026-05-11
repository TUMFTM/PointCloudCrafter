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
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>
#include <pcl/memory.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <urdf/model.h>

#include <Eigen/Eigen>
#include <filesystem>  // NOLINT
#include <functional>
#include <limits>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sstream>
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
 * @param cfg Configuration
 */
Rosbag::Rosbag(const config::RosbagConfig & cfg)
: cfg_(cfg),
  tf2_buffer_(std::make_shared<rclcpp::Clock>()),
  logger_(rclcpp::get_logger("pointcloudcrafter_rosbag")),
  num_sensors_(cfg_.topics.size()),
  reader_(cfg_.bag_path)
{
  // Check for number of topics
  if (cfg_.topics.size() > 4) {
    throw std::runtime_error("Only a maximum of 4 topics are supported");
  }
  // Initialize rosbag writer if saving to rosbag is enabled
  if (cfg_.save_rosbag && !cfg_.rosbag_topic.empty()) {
    rosbag_writer_ = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();
     rosbag2_storage::StorageOptions writer_options;
    writer_options.storage_id = "mcap";
    std::filesystem::path out_path(cfg_.out_dir);
    writer_options.uri = (out_path / "processed_rosbag").string();
    rosbag_writer_->open(writer_options, rosbag2_cpp::ConverterOptions());
    rosbag2_storage::TopicMetadata topic_metadata;
    topic_metadata.name = cfg_.rosbag_topic;
    topic_metadata.type = "sensor_msgs/msg/PointCloud2";
    topic_metadata.serialization_format = "cdr";
    rosbag_writer_->create_topic(topic_metadata);
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

  if (!cfg.urdf_file.empty()) {
    urdf::Model urdf_model;
    if (!urdf_model.initFile(cfg.urdf_file)) {
      RCLCPP_WARN(logger_, "Could not open URDF file: %s", cfg.urdf_file.c_str());
    } else {
      for (auto const & joint_kv : urdf_model.joints_) {
        urdf::JointSharedPtr joint = joint_kv.second;
        geometry_msgs::msg::TransformStamped t;
        t.header.frame_id = joint->parent_link_name;
        t.child_frame_id = joint->child_link_name;

        auto xyz = joint->parent_to_joint_origin_transform.position;
        auto rpy = joint->parent_to_joint_origin_transform.rotation;

        double roll, pitch, yaw;
        rpy.getRPY(roll, pitch, yaw);  // extract Euler angles from urdf::Rotation

        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);

        t.transform.translation.x = xyz.x;
        t.transform.translation.y = xyz.y;
        t.transform.translation.z = xyz.z;
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf2_buffer_.setTransform(t, "urdf", true);
      }
    }
  }

  // Load transforms from file
  if (!cfg_.transform_file.empty()) {
    this->file_transforms_ = tools::utils::load_transforms_from_file(cfg_.transform_file);
  }

  // Init out directory
  if (!std::filesystem::exists(cfg_.out_dir)) {
    std::filesystem::create_directories(cfg_.out_dir);
  }
}

/**
 * @brief Callback for TF messages
 * @param msg TF message
 */
void Rosbag::tf_callback(const tools::RosbagReaderMsg<tf2_msgs::msg::TFMessage> & msg)
{
  for (auto & tf : msg.ros_msg.transforms) {
    if (tf.header.frame_id == tf.child_frame_id) {
      continue;
    }
    tf2_buffer_.setTransform(tf, "bag", msg.bag_msg.topic_name == "/tf_static");
  }
}
/**
 * @brief Callback for synchronized pointclouds
 * @param pc1 Pointcloud message 1
 * @param pc2 Pointcloud message 2
 * @param pc3 Pointcloud message 3
 * @param pc4 Pointcloud message 4
 */
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
/**
 * @brief Process synchronized pointclouds
 * @param pc_msgs Vector of pointcloud messages
 */
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

  auto save_fmt = cfg_.get_save_format();
  std::string ext = pointcloudcrafter::tools::formats::format_to_extension(save_fmt);

  // Grid split path
  if (cfg_.split_grid_size > 0.0) {
    std::string split_dir = cfg_.out_dir + "/" + name;
    if (!std::filesystem::exists(split_dir)) {
      std::filesystem::create_directories(split_dir);
    }

    auto cells = modifier.split(cfg_.split_grid_size);

    auto format_coord = [](double val) -> std::string {
      if (val == std::floor(val)) {
        std::ostringstream oss;
        oss << static_cast<int64_t>(val);
        return oss.str();
      }
      std::ostringstream oss;
      oss << val;
      return oss.str();
    };

    for (auto & [key, cell] : cells) {
      double cx = key.first * cfg_.split_grid_size;
      double cy = key.second * cfg_.split_grid_size;
      std::string cell_path = split_dir + "/" + format_coord(cx) + "_" + format_coord(cy) + ext;
      if (!cell.save(cell_path, save_fmt)) {
        RCLCPP_ERROR(logger_, "Failed to save cell: %s", cell_path.c_str());
      }
    }

    pointcloudmodifierlib::Modifier::writeGridMetadata(
      cells, cfg_.split_grid_size,
      split_dir + "/pointcloud_map_metadata.yaml", ext);
    return;
  }

  // Normal single-frame save path
  if (!modifier.save(cfg_.out_dir + "/" + name + ext, save_fmt)) {
    RCLCPP_ERROR(logger_, "Failed to save: %s", (cfg_.out_dir + "/" + name + ext).c_str());
    return;
  }

  if (cfg_.save_rosbag && !cfg_.rosbag_topic.empty()) {
    writePCLToRosbag(
      *modifier.getOutputCloud(), cfg_.rosbag_topic, cfg_.target_frame, rclcpp::Time(ts));
  }

  // Save timestamps if enabled
  if (cfg_.timestamps) {
    modifier.timestampAnalyzer(cfg_.out_dir + "/" + name + "_stamps.txt");
  }
}
/**
 * @brief Transform a pointcloud given the transform from either the file or the TF buffer
 * @param msg_in Input pointcloud
 * @param msg_out Output pointcloud
 */
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
/**
 * @brief Write a point cloud to a rosbag
 * @param cloud Point cloud to write
 * @param topic ROS topic to write to
 * @param frame_id Frame ID to set in the message header
 * @param timestamp Timestamp to set in the message header
 * @param writer Rosbag writer to use for writing the message
 * @return True if successful, false otherwise
 */
bool Rosbag::writePCLToRosbag(
  const pcl::PCLPointCloud2 & cloud,
  const std::string & topic,
  const std::string & frame_id,
  const rclcpp::Time & timestamp)
{
  if (!rosbag_writer_) {
    RCLCPP_ERROR(logger_, "Rosbag writer not initialized");
    return false;
  }
  sensor_msgs::msg::PointCloud2 msg;
  pcl_conversions::fromPCL(cloud, msg);
  msg.header.stamp = timestamp;
  msg.header.frame_id = frame_id;

  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
  rclcpp::SerializedMessage serialized_ros_msg;
  serialization.serialize_message(&msg, &serialized_ros_msg);

  rosbag2_storage::SerializedBagMessageSharedPtr serialized_msg =
    std::make_shared<rosbag2_storage::SerializedBagMessage>();
  serialized_msg->topic_name = topic;
  serialized_msg->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
    new rcutils_uint8_array_t(serialized_ros_msg.release_rcl_serialized_message()),
    [](rcutils_uint8_array_t * msg) {
      rcutils_uint8_array_fini(msg);
      delete msg;
    });
  serialized_msg->recv_timestamp = timestamp.nanoseconds();
  serialized_msg->send_timestamp = timestamp.nanoseconds();

  rosbag_writer_->write(serialized_msg);
  return true;
}
}  // namespace pointcloudcrafter
