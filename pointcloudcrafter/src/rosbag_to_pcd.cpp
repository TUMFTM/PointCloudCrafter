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

#include "pointcloudcrafter/rosbag_to_pcd.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/util/Constants.h>
#include <fmt/format.h>
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
#include <yaml-cpp/node/parse.h>

#include <Eigen/Eigen>
#include <cstddef>
#include <cstdint>
#include <filesystem>
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
#include <tf2_msgs/msg/detail/tf_message__struct.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <vector>

#include "pointcloudcrafter/filter_subscriber.hpp"
#include "pointcloudcrafter/rosbag_reader.hpp"
#include "pointcloudcrafter/utils.hpp"
namespace pointcloudcrafter {
constexpr float COLORS[] = {0.0, 1.0, 0.333, 0.666};

// global variables that will be populated by CLI arguments
std::string bag_path;  // NOLINT
std::vector<std::string> topic_names;
std::string out_dir;                // NOLINT
std::string target_frame = "";      // NOLINT
std::string sensor_number_field{};  // NOLINT
std::string transform_file{};       // NOLINT
int64_t max_frames = -1;
int64_t skip_frames = 0;
int64_t stride_frames = 1;
bool sequential_names = false;
bool bag_time = false;
std::vector<float> geometric_filtering{};
bool pie_filter = false;

using PointCloud = pcl::PointCloud<pt::PointXYZIT>;
RosbagToPcd::RosbagToPcd()
    : reader_(bag_path),
      tf2_buffer_(std::make_shared<rclcpp::Clock>()),
      logger_(rclcpp::get_logger("rosbag_to_pcd")),
      num_sensors_(topic_names.size()) {
    if (topic_names.size() > 4) {
        throw std::runtime_error("Only a maximum of 4 topics are supported");
    }
    for (size_t i = 0; i < 4; i++) {
        std::string topic = (i < num_sensors_) ? topic_names[i] : topic_names[0];
        subscribers_.push_back(
            std::make_unique<tools::BagSubscriber<sensor_msgs::msg::PointCloud2>>(topic, reader_,
                                                                                  bag_time));
    }

    auto tf_callback = std::bind(&RosbagToPcd::transform_callback, this, std::placeholders::_1);
    reader_.add_listener<tf2_msgs::msg::TFMessage>("/tf", tf_callback);
    reader_.add_listener<tf2_msgs::msg::TFMessage>("/tf_static", tf_callback);

    synchronizer_ =
        std::make_unique<PCTimeSynchronizer>(ApproxSyncPolicy(20), *subscribers_[0],
                                             *subscribers_[1], *subscribers_[2], *subscribers_[3]);
    sync_connection_ = synchronizer_->registerCallback(
        std::bind(&RosbagToPcd::pointcloud_callback_sync, this, std::placeholders::_1,
                  std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));

    tf2_buffer_.setUsingDedicatedThread(true);

    std::filesystem::create_directories(out_dir + "/pcd");
    std::filesystem::create_directories(out_dir + "/times");

    if (!transform_file.empty()) {
        transform_config_ = YAML::LoadFile(transform_file);
    }
}
void RosbagToPcd::run() {
    reader_.process();

    tools::utils::save_timestamps(out_dir + "/times/lidar_timestamps.txt", timestamps_lidar_);
}
std::string RosbagToPcd::make_filename(uint64_t timestamp) const {
    std::string basename;
    if (sequential_names) {
        char tmp[20];
        std::snprintf(tmp, sizeof(tmp), "%06ld", loaded_frames_);
        basename = tmp;
    } else {
        auto ts = tools::utils::timestamp_to_ros(timestamp);
        basename = fmt::format("{}_{:09}", ts.sec, ts.nanosec);
    }

    return out_dir + "/pcd/" + basename + ".pcd";
}
void RosbagToPcd::save(uint64_t timestamp, const pcl::PCLPointCloud2::Ptr &pc) {
    if (!geometric_filtering.empty()) {
        pcl::CropBox<pcl::PCLPointCloud2> crop_box;

        // filter points inside rectangular box
        crop_box.setMin(
            {geometric_filtering[0], geometric_filtering[1], geometric_filtering[2], 1.0});
        crop_box.setMax(
            {geometric_filtering[3], geometric_filtering[4], geometric_filtering[5], 1.0});
        crop_box.setInputCloud(pc);
        crop_box.setNegative(true);
        crop_box.filter(*pc);
    }

    if (pie_filter) {
        pcl::PCLPointCloud2 filtered_cloud;
        filtered_cloud.header = pc->header;
        filtered_cloud.height = 1;
        filtered_cloud.width = 0;
        filtered_cloud.fields = pc->fields;
        filtered_cloud.point_step = pc->point_step;
        filtered_cloud.row_step = 0;
        filtered_cloud.is_bigendian = pc->is_bigendian;
        filtered_cloud.is_dense = pc->is_dense;
        filtered_cloud.data.clear();

        for (size_t i = 0; i < pc->data.size(); i += pc->point_step) {
            float x, y;
            for (size_t j = 0; j < pc->fields.size(); ++j) {
                pcl::PCLPointField &field = pc->fields[j];
                size_t point_offset = i + field.offset;
                if (field.name == "x") {
                    if (field.datatype == pcl::PCLPointField::FLOAT32) {
                        memcpy(&x, &pc->data[point_offset], sizeof(float));
                    }
                } else if (field.name == "y") {
                    if (field.datatype == pcl::PCLPointField::FLOAT32) {
                        memcpy(&y, &pc->data[point_offset], sizeof(float));
                    }
                } else {
                    break;
                }
            }
            auto point_angle = std::atan2(y, x - 1.5);
            auto segment_angle = M_PI * (300.0f / 360.0f);

            if (std::abs(point_angle) <= segment_angle) {
                filtered_cloud.data.insert(filtered_cloud.data.end(), pc->data.begin() + i,
                                           pc->data.begin() + i + pc->point_step);
                filtered_cloud.width++;
                filtered_cloud.row_step += pc->point_step;
            }
        }
        *pc = filtered_cloud;
    }
    // This part of the code is solely dedicated to LUMINAR, as they are not able to keep their
    // driver to the PointCloud2 convention. LUMINAR appends a default PointField with '',0,0,0 to
    // the end of each message.
    for (size_t i = 0; i < pc->fields.size(); ++i) {
        if (pc->fields[i].count == 0) {
            PointCloud pc_xyzit{};
            pcl::fromPCLPointCloud2(*pc, pc_xyzit);
            writer_.writeBinary(make_filename(timestamp), pc_xyzit);
        } else {
            // pcl::PointCloud<pt::PointXYZITd> pc_xyzitd{};
            // pcl::fromPCLPointCloud2(*pc, pc_xyzitd);
            // std::cout << std::fixed << std::setprecision(9); std::cout <<
            // pc_xyzitd.points[i].timestamp
            // << std::endl;
            // writer_.writeBinary(make_filename(timestamp), pc_xyzitd);
            writer_.writeBinary(make_filename(timestamp), *pc);
        }
    }

    timestamps_lidar_.push_back(timestamp);

    stride_frames_ = stride_frames - 1;

    loaded_frames_++;
    if (max_frames > 0 && loaded_frames_ >= max_frames) {
        reader_.set_state(false);
    }
}
void RosbagToPcd::transform_callback(const tools::RosbagReaderMsg<tf2_msgs::msg::TFMessage> &msg) {
    for (auto &tf : msg.ros_msg.transforms) {
        if (tf.header.frame_id == tf.child_frame_id) {
            continue;
        }
        tf2_buffer_.setTransform(tf, "bag", msg.bag_msg.topic_name == "/tf_static");
    }
}
void RosbagToPcd::pointcloud_callback_sync(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pc1,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pc2,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pc3,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pc4) {
    if (skip_frames > 0) {
        skip_frames--;
        return;
    }

    if (stride_frames_ > 0) {
        stride_frames_--;
        return;
    }

    // attention: size of vector may not always equal number of sensors!
    // last items may be dummy
    std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr> pc_msgs{pc1, pc2, pc3, pc4};
    pc_msgs.resize(num_sensors_);

    process_merge_and_save(pc_msgs);
}
void RosbagToPcd::transform_pc(const sensor_msgs::msg::PointCloud2 &msg_in,
                               sensor_msgs::msg::PointCloud2 &msg_out) {
    Eigen::Affine3d transformation(Eigen::Affine3d::Identity());

    if (auto node = transform_config_["override_" + msg_in.header.frame_id]) {
        auto entries = node.as<std::vector<double>>();
        Eigen::Matrix<double, 4, 4, Eigen::RowMajor> override_transform(entries.data());
        transformation = override_transform;
    } else if (!target_frame.empty()) {
        geometry_msgs::msg::Transform tf =
            tf2_buffer_.lookupTransform(target_frame, msg_in.header.frame_id, rclcpp::Time{0})
                .transform;

        Eigen::Quaterniond rot{tf.rotation.w, tf.rotation.x, tf.rotation.y, tf.rotation.z};
        transformation =
            Eigen::Translation3d(tf.translation.x, tf.translation.y, tf.translation.z) * rot;

        if (auto node = transform_config_["extra_" + msg_in.header.frame_id]) {
            auto entries = node.as<std::vector<double>>();
            Eigen::Matrix<double, 4, 4, Eigen::RowMajor> extra_transform(entries.data());
            transformation *= extra_transform;
        }
    }

    tools::utils::transform_pointcloud2(transformation.cast<float>(), msg_in, msg_out);
}
void RosbagToPcd::process_merge_and_save(
    std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr> &pc_msgs) {
    uint64_t base_time = std::numeric_limits<uint64_t>::max();
    // find the smallest timestamp
    for (auto &msg : pc_msgs) {
        uint64_t time = tools::utils::timestamp_from_ros(msg->header.stamp);
        if (time < base_time) {
            base_time = time;
        }
    }

    auto merged_pc = pcl::make_shared<pcl::PCLPointCloud2>();
    for (size_t i = 0; i < pc_msgs.size(); i++) {
        const sensor_msgs::msg::PointCloud2 &msg = *pc_msgs[i];
        sensor_msgs::msg::PointCloud2 msg_transformed;
        transform_pc(msg, msg_transformed);

        try {
            // adjust time information to be relative to base_time
            uint64_t time_offset = tools::utils::timestamp_from_ros(msg.header.stamp) - base_time;
            for (sensor_msgs::PointCloud2Iterator<uint64_t> it(msg_transformed, "time_us");
                 it != it.end(); ++it) {
                *it += time_offset;
            }
        } catch (std::runtime_error &) {
            // do nothing if cloud has no field t
        }

        // write the sensor number if specified
        if (!sensor_number_field.empty()) {
            for (sensor_msgs::PointCloud2Iterator<float> it(msg_transformed, sensor_number_field);
                 it != it.end(); ++it) {
                *it = COLORS[i % 4];
            }
        }

        pcl::PCLPointCloud2 pc;
        pcl_conversions::toPCL(msg_transformed, pc);
        *merged_pc += pc;
    }

    save(base_time, merged_pc);
}
}  // namespace pointcloudcrafter
