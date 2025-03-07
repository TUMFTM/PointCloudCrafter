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

#define PCL_NO_PRECOMPILE
#pragma once

#include <message_filters/connection.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/buffer.h>
#include <yaml-cpp/yaml.h>

#include <cstdint>
#include <gps_msgs/msg/gps_fix.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <tf2_msgs/msg/tf_message.hpp>
#include <vector>

#include "rosbag_reader.hpp"
#include "pointcloudcrafter/filter_subscriber.hpp"
namespace tum::mapping {
extern std::string bag_path;
extern std::vector<std::string> topic_names;
extern std::string out_dir;
extern std::string target_frame;
extern std::string sensor_number_field;
extern std::string transform_file;
extern int64_t max_frames;
extern int64_t skip_frames;
extern int64_t stride_frames;
extern bool sequential_names;
extern bool bag_time;
extern std::vector<float> geometric_filtering;
extern bool pie_filter;
using ApproxSyncPolicy =
    message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2,
                                                    sensor_msgs::msg::PointCloud2,
                                                    sensor_msgs::msg::PointCloud2,
                                                    sensor_msgs::msg::PointCloud2>;
using PCTimeSynchronizer = message_filters::Synchronizer<ApproxSyncPolicy>;
class RosbagToPcd {
    tools::RosbagReader reader_;
    pcl::PCDWriter writer_;
    tf2_ros::Buffer tf2_buffer_;
    rclcpp::Logger logger_;
    std::vector<std::unique_ptr<tools::BagSubscriber<sensor_msgs::msg::PointCloud2>>> subscribers_;
    std::unique_ptr<PCTimeSynchronizer> synchronizer_;
    message_filters::Connection sync_connection_{};

    YAML::Node transform_config_;

    size_t num_sensors_;

    std::vector<uint64_t> timestamps_lidar_;

    int64_t loaded_frames_{0};
    int64_t stride_frames_{0};

public:
    RosbagToPcd();

    void run();

protected:
    std::string make_filename(uint64_t timestamp) const;

    void save(uint64_t timestamp, const pcl::PCLPointCloud2::Ptr &pc);

    void transform_callback(const tools::RosbagReaderMsg<tf2_msgs::msg::TFMessage> &msg);

    void pointcloud_callback_sync(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pc1,
                                  const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pc2,
                                  const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pc3,
                                  const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pc4);

    void process_merge_and_save(
        std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr> &pc_msgs);

    void transform_pc(const sensor_msgs::msg::PointCloud2 &msg_in,
                      sensor_msgs::msg::PointCloud2 &msg_out);

    void circleSegmentFilter(float &circ_angle, float &offset);
};
// Dedicated to LUMINAR
class pt {
public:
    struct PointXYZIT {
        PCL_ADD_POINT4D;
        float intensity;
        std::uint32_t timestamp;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    } EIGEN_ALIGN16;
    struct PointXYZITd {
        PCL_ADD_POINT4D;
        float intensity;
        double timestamp;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    } EIGEN_ALIGN16;
};
}  // namespace tum::mapping

POINT_CLOUD_REGISTER_POINT_STRUCT(tum::mapping::pt::PointXYZIT,
                                  (float, x, x)(float, y, y)(float, z, z)(float,
                                                                          intensity,
                                                                          intensity)(std::uint32_t,
                                                                                     timestamp,
                                                                                     time_us))

POINT_CLOUD_REGISTER_POINT_STRUCT(tum::mapping::pt::PointXYZITd,
                                  (float, x, x)(float, y, y)(float, z, z)(
                                      float, intensity, intensity)(double, timestamp, timestamp))
