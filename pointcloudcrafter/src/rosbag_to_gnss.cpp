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

#include "pointcloudcrafter/rosbag_to_gnss.hpp"

#include <fmt/format.h>

#include <GeographicLib/GeoCoords.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tuple>
#include <vector>

#include "pointcloudcrafter/rosbag_reader.hpp"
#include "pointcloudcrafter/utils.hpp"
namespace tum::mapping {

// global variables that will be populated by CLI arguments
std::string bag_path{};              // NOLINT
std::string topic_name{};            // NOLINT
std::string msg_type{};              // NOLINT
std::string out_dir{};               // NOLINT
std::vector<double> origin_wgs84{};  // NOLINT
bool convert_utm(false);
bool dump_gnss(false);
std::string gnss_antenna{};  // NOLINT
RosbagToGnss::RosbagToGnss()
    : reader_(bag_path),
      tf2_buffer_(std::make_shared<rclcpp::Clock>()),
      logger_(rclcpp::get_logger("rosbag_to_gnss")) {
    if (!origin_wgs84.empty() && origin_wgs84.size() != 3) {
        throw std::runtime_error("Invalid number of origin arguments! Valid: [lat lon hgt]!");
    }

    if (msg_type == "vectornav_msgs/msg/GpsGroup") {
        auto gpsgroup_callback =
            std::bind(&RosbagToGnss::gpsgroup_callback, this, std::placeholders::_1);
        reader_.add_listener<vectornav_msgs::msg::GpsGroup>(topic_name, gpsgroup_callback);
    } else if (msg_type == "vectornav_msgs/msg/InsGroup") {
        auto insgroup_callback =
            std::bind(&RosbagToGnss::insgroup_callback, this, std::placeholders::_1);
        reader_.add_listener<vectornav_msgs::msg::InsGroup>(topic_name, insgroup_callback);
    } else if (msg_type == "novatel_oem7_msgs/msg/BESTPOS") {
        auto bestpos_callback =
            std::bind(&RosbagToGnss::bestpos_callback, this, std::placeholders::_1);
        reader_.add_listener<novatel_oem7_msgs::msg::BESTPOS>(topic_name, bestpos_callback);
    } else if (msg_type == "novatel_oem7_msgs/msg/BESTGNSSPOS") {
        auto bestgnsspos_callback =
            std::bind(&RosbagToGnss::bestgnsspos_callback, this, std::placeholders::_1);
        reader_.add_listener<novatel_oem7_msgs::msg::BESTGNSSPOS>(topic_name, bestgnsspos_callback);
    } else if (msg_type == "sensor_msgs/msg/NavSatFix") {
        auto navsatfix_callback =
            std::bind(&RosbagToGnss::navsatfix_callback, this, std::placeholders::_1);
        reader_.add_listener<sensor_msgs::msg::NavSatFix>(topic_name, navsatfix_callback);
    } else if (msg_type == "nav_msgs/msg/Odometry") {
        auto odometry_callback =
            std::bind(&RosbagToGnss::odometry_callback, this, std::placeholders::_1);
        reader_.add_listener<nav_msgs::msg::Odometry>(topic_name, odometry_callback);
    } else {
        throw std::runtime_error("Unsupported GNSS msg type!");
    }

    if (!gnss_antenna.empty()) {
        auto tf_callback =
            std::bind(&RosbagToGnss::transform_callback, this, std::placeholders::_1);
        reader_.add_listener<tf2_msgs::msg::TFMessage>("/tf", tf_callback);
        reader_.add_listener<tf2_msgs::msg::TFMessage>("/tf_static", tf_callback);

        tf2_buffer_.setUsingDedicatedThread(true);
    }

    topic_name_replacement = topic_name;
    std::replace(topic_name_replacement.begin(), topic_name_replacement.end(), '/', '_');

    out_dir = (out_dir.back() == '/') ? out_dir : out_dir + "/";
    std::filesystem::create_directories(out_dir + topic_name_replacement + "/data");
}
void RosbagToGnss::run() {
    reader_.process();
    tools::utils::save_timestamps(out_dir + topic_name_replacement + "timestamps.txt", timestamps_);
}
void RosbagToGnss::gpsgroup_callback(
    const tools::RosbagReaderMsg<vectornav_msgs::msg::GpsGroup> &msg) {
    auto gpsmsg = msg.ros_msg;
    auto ts = msg.ros_msg.header.stamp;
    auto filename = fmt::format("{}/{}/data/{:010d}_{:09d}.txt", out_dir, topic_name_replacement,
                                ts.sec, ts.nanosec);

    RosbagToGnss::GnssPos pos;
    pos.x = gpsmsg.poslla.x;
    pos.y = gpsmsg.poslla.y;
    pos.z = gpsmsg.poslla.z;
    pos.x_stdev = gpsmsg.posu.x;
    pos.y_stdev = gpsmsg.posu.y;
    pos.z_stdev = gpsmsg.posu.z;

    if (!gnss_antenna.empty()) {
        transform_base_link(pos, gnss_antenna);
    }

    if (convert_utm) {
        convert_to_utm(pos);
    } else if (!origin_wgs84.empty()) {
        relative_to_origin(pos, origin_wgs84);
    }

    if (dump_gnss) {
        auto fn = fmt::format("{}/{}.txt", out_dir, topic_name_replacement);
        save_latlonalt(fn, pos);
    }
    save_latlonalt(filename, pos);

    timestamps_.push_back(tools::utils::timestamp_from_ros(msg.ros_msg.header.stamp));
}
void RosbagToGnss::insgroup_callback(
    const tools::RosbagReaderMsg<vectornav_msgs::msg::InsGroup> &msg) {
    auto gpsmsg = msg.ros_msg;
    auto ts = msg.ros_msg.header.stamp;
    auto filename = fmt::format("{}/{}/data/{:010d}_{:09d}.txt", out_dir, topic_name_replacement,
                                ts.sec, ts.nanosec);

    RosbagToGnss::GnssPos pos;
    pos.x = gpsmsg.poslla.x;
    pos.y = gpsmsg.poslla.y;
    pos.z = gpsmsg.poslla.z;
    pos.x_stdev = gpsmsg.posu;
    pos.y_stdev = gpsmsg.posu;
    pos.z_stdev = gpsmsg.posu;

    if (!gnss_antenna.empty()) {
        transform_base_link(pos, gnss_antenna);
    }

    if (convert_utm) {
        convert_to_utm(pos);
    } else if (!origin_wgs84.empty()) {
        relative_to_origin(pos, origin_wgs84);
    }

    if (dump_gnss) {
        auto fn = fmt::format("{}/{}.txt", out_dir, topic_name_replacement);
        save_latlonalt(fn, pos);
    }
    save_latlonalt(filename, pos);

    timestamps_.push_back(tools::utils::timestamp_from_ros(msg.ros_msg.header.stamp));
}
void RosbagToGnss::bestpos_callback(
    const tools::RosbagReaderMsg<novatel_oem7_msgs::msg::BESTPOS> &msg) {
    auto gpsmsg = msg.ros_msg;
    auto ts = msg.ros_msg.header.stamp;
    auto filename = fmt::format("{}/{}/data/{:010d}_{:09d}.txt", out_dir, topic_name_replacement,
                                ts.sec, ts.nanosec);

    RosbagToGnss::GnssPos pos;
    pos.x = gpsmsg.lat;
    pos.y = gpsmsg.lon;
    pos.z = gpsmsg.hgt;
    pos.x_stdev = gpsmsg.lat_stdev;
    pos.y_stdev = gpsmsg.lon_stdev;
    pos.z_stdev = gpsmsg.hgt_stdev;

    if (!gnss_antenna.empty()) {
        transform_base_link(pos, gnss_antenna);
    }

    if (convert_utm) {
        convert_to_utm(pos);
    } else if (!origin_wgs84.empty()) {
        relative_to_origin(pos, origin_wgs84);
    }

    if (dump_gnss) {
        auto fn = fmt::format("{}/{}.txt", out_dir, topic_name_replacement);
        save_latlonalt(fn, pos);
    }
    save_latlonalt(filename, pos);

    timestamps_.push_back(tools::utils::timestamp_from_ros(msg.ros_msg.header.stamp));
}
void RosbagToGnss::bestgnsspos_callback(
    const tools::RosbagReaderMsg<novatel_oem7_msgs::msg::BESTGNSSPOS> &msg) {
    auto gpsmsg = msg.ros_msg;
    auto ts = msg.ros_msg.header.stamp;
    auto filename = fmt::format("{}/{}/data/{:010d}_{:09d}.txt", out_dir, topic_name_replacement,
                                ts.sec, ts.nanosec);

    RosbagToGnss::GnssPos pos;
    pos.x = gpsmsg.lat;
    pos.y = gpsmsg.lon;
    pos.z = gpsmsg.hgt;
    pos.x_stdev = gpsmsg.lat_stdev;
    pos.y_stdev = gpsmsg.lon_stdev;
    pos.z_stdev = gpsmsg.hgt_stdev;

    if (!gnss_antenna.empty()) {
        transform_base_link(pos, gnss_antenna);
    }

    if (convert_utm) {
        convert_to_utm(pos);
    } else if (!origin_wgs84.empty()) {
        relative_to_origin(pos, origin_wgs84);
    }

    if (dump_gnss) {
        auto fn = fmt::format("{}/{}.txt", out_dir, topic_name_replacement);
        save_latlonalt(fn, pos);
    }
    save_latlonalt(filename, pos);

    timestamps_.push_back(tools::utils::timestamp_from_ros(msg.ros_msg.header.stamp));
}
void RosbagToGnss::navsatfix_callback(
    const tools::RosbagReaderMsg<sensor_msgs::msg::NavSatFix> &msg) {
    auto gpsmsg = msg.ros_msg;
    auto ts = msg.ros_msg.header.stamp;
    auto filename = fmt::format("{}/{}/data/{:010d}_{:09d}.txt", out_dir, topic_name_replacement,
                                ts.sec, ts.nanosec);

    RosbagToGnss::GnssPos pos;
    pos.x = gpsmsg.latitude;
    pos.y = gpsmsg.longitude;
    pos.z = gpsmsg.altitude;
    pos.x_stdev = gpsmsg.position_covariance[0];
    pos.y_stdev = gpsmsg.position_covariance[4];
    pos.z_stdev = gpsmsg.position_covariance[8];

    if (!gnss_antenna.empty()) {
        transform_base_link(pos, gnss_antenna);
    }

    if (convert_utm) {
        convert_to_utm(pos);
    } else if (!origin_wgs84.empty()) {
        relative_to_origin(pos, origin_wgs84);
    }

    if (dump_gnss) {
        auto fn = fmt::format("{}/{}.txt", out_dir, topic_name_replacement);
        save_latlonalt(fn, pos);
    }
    save_latlonalt(filename, pos);

    timestamps_.push_back(tools::utils::timestamp_from_ros(msg.ros_msg.header.stamp));
}
void RosbagToGnss::odometry_callback(const tools::RosbagReaderMsg<nav_msgs::msg::Odometry> &msg) {
    auto gpsmsg = msg.ros_msg;
    auto ts = msg.ros_msg.header.stamp;
    auto filename = fmt::format("{}/{}/data/{:010d}_{:09d}.txt", out_dir, topic_name_replacement,
                                ts.sec, ts.nanosec);

    RosbagToGnss::GnssPos pos;
    pos.x = gpsmsg.pose.pose.position.x;
    pos.y = gpsmsg.pose.pose.position.y;
    pos.z = gpsmsg.pose.pose.position.z;
    pos.x_stdev = gpsmsg.pose.covariance[0];
    pos.y_stdev = gpsmsg.pose.covariance[7];
    pos.z_stdev = gpsmsg.pose.covariance[14];

    if (!gnss_antenna.empty()) {
        throw std::runtime_error(
            "This / these function(s) are not available with nav/msg/odometry!");
    }

    if (convert_utm || !origin_wgs84.empty()) {
        throw std::runtime_error(
            "This / these function(s) are not available with nav/msg/odometry!");
    }

    if (dump_gnss) {
        auto fn = fmt::format("{}/{}.txt", out_dir, topic_name_replacement);
        save_latlonalt(fn, pos);
    }
    save_latlonalt(filename, pos);

    timestamps_.push_back(tools::utils::timestamp_from_ros(msg.ros_msg.header.stamp));
}
void RosbagToGnss::save_latlonalt(const std::string &filename, GnssPos &pos) {
    std::ofstream oxts_file;
    oxts_file.open(filename, std::ios_base::app);
    oxts_file << fmt::format("{} {} {} {} {} {}\n", pos.x, pos.y, pos.z, pos.x_stdev, pos.y_stdev,
                             pos.z_stdev);
}
void RosbagToGnss::transform_callback(const tools::RosbagReaderMsg<tf2_msgs::msg::TFMessage> &msg) {
    for (auto &tf : msg.ros_msg.transforms) {
        if (tf.header.frame_id == tf.child_frame_id) {
            continue;
        }
        tf2_buffer_.setTransform(tf, "bag", msg.bag_msg.topic_name == "/tf_static");
        // std::cout << tf.header.frame_id << "->" << tf.child_frame_id << std::endl;
    }
}
void RosbagToGnss::transform_base_link(GnssPos &pos, std::string &frame_id) {
    // std::cout << frame_id << std::endl;
    geometry_msgs::msg::Transform tf_s{};
    geometry_msgs::msg::Transform tf{};

    try {
        // Check if transform is available
        tf_s = tf2_buffer_.lookupTransform(frame_id, "base_link", tf2::TimePointZero).transform;
        tf = tf2_buffer_.lookupTransform("local_cartesian", "base_link", tf2::TimePointZero)
                 .transform;
        // RCLCPP_INFO(logger_, "Transformations available!");
    } catch (tf2::TransformException &ex) {
        // If the transform is not available, catch the exception and continue waiting
        RCLCPP_INFO(logger_, "Transform not available: %s", ex.what());
    }
    // Set transformation
    Eigen::Quaterniond rot(tf.rotation.w, tf.rotation.x, tf.rotation.y, tf.rotation.z);
    Eigen::Vector3d trans_static(tf_s.translation.x, tf_s.translation.y, tf_s.translation.z);

    // std::cout << "trans: " << trans_static << std::endl;
    Eigen::Vector3d trans = rot * trans_static;

    // std::cout << "trans_dyn: " << trans << std::endl;
    GeographicLib::LocalCartesian c(pos.x, pos.y, pos.z);
    c.Reverse(trans(0), trans(1), trans(2), pos.x, pos.y, pos.z);
}
void RosbagToGnss::convert_to_utm(GnssPos &pos) {
    GeographicLib::GeoCoords c(pos.x, pos.y);
    pos.y = c.Northing();
    pos.x = c.Easting();
    std::swap(pos.x_stdev, pos.y_stdev);
}
void RosbagToGnss::relative_to_origin(GnssPos &pos, std::vector<double> &origin) {
    GeographicLib::LocalCartesian c(origin[0], origin[1], origin[2]);
    c.Forward(pos.x, pos.y, pos.z, pos.x, pos.y, pos.z);
}
}  // namespace tum::mapping
