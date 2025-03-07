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

#include <tf2_ros/buffer.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <string>
#include <tf2_msgs/msg/detail/tf_message__struct.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "novatel_oem7_msgs/msg/bestpos.hpp"
#include "novatel_oem7_msgs/msg/bestgnsspos.hpp"
#include "rosbag_reader.hpp"
#include "pointcloudcrafter/rosbag_reader.hpp"
#include "vectornav_msgs/msg/gps_group.hpp"
#include "vectornav_msgs/msg/ins_group.hpp"
namespace tum::mapping {
extern std::string bag_path;
extern std::string topic_name;
extern std::string msg_type;
extern std::string out_dir;
extern std::vector<double> origin_wgs84;
extern bool convert_utm;
extern bool dump_gnss;
extern std::string gnss_antenna;
class RosbagToGnss {
    tools::RosbagReader reader_;
    tf2_ros::Buffer tf2_buffer_;
    rclcpp::Logger logger_;

    std::vector<uint64_t> timestamps_;

    int64_t loaded_frames_{0};
    int64_t stride_frames_{0};

    std::string topic_name_replacement;

public:
    RosbagToGnss();
    void run();
    class GnssPos {
    public:
        double x{0.0}, y{0.0}, z{0.0}, x_stdev{0.0}, y_stdev{0.0}, z_stdev{0.0};
    };
    void save_latlonalt(const std::string &filename, GnssPos &pos);
    void convert_to_utm(GnssPos &pos);
    void relative_to_origin(GnssPos &pos, std::vector<double> &origin);

protected:
    void gpsgroup_callback(const tools::RosbagReaderMsg<vectornav_msgs::msg::GpsGroup> &msg);
    void insgroup_callback(const tools::RosbagReaderMsg<vectornav_msgs::msg::InsGroup> &msg);
    void bestpos_callback(const tools::RosbagReaderMsg<novatel_oem7_msgs::msg::BESTPOS> &msg);
    void bestgnsspos_callback(
        const tools::RosbagReaderMsg<novatel_oem7_msgs::msg::BESTGNSSPOS> &msg);
    void navsatfix_callback(const tools::RosbagReaderMsg<sensor_msgs::msg::NavSatFix> &msg);
    void odometry_callback(const tools::RosbagReaderMsg<nav_msgs::msg::Odometry> &msg);
    void transform_callback(const tools::RosbagReaderMsg<tf2_msgs::msg::TFMessage> &msg);
    void transform_base_link(GnssPos &pos, std::string &frame_id);
};
}  // namespace tum::mapping
