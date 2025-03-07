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

#include <functional>
#include <memory>
#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "message_filters/message_event.h"
#include "message_filters/simple_filter.h"
namespace pointcloudcrafter::tools
{
template <typename T>
struct RosbagReaderMsg
{
  const rosbag2_storage::SerializedBagMessage & bag_msg;
  const T & ros_msg;
};
class SerializedMessageHandlerBase
{
public:
  virtual void handle_serialized_message(
    const rosbag2_storage::SerializedBagMessage & bag_msg,
    const rclcpp::SerializedMessage & ser_msg) = 0;
};
template <typename T>
class SerializedMessageHandler : public SerializedMessageHandlerBase
{
  rclcpp::Serialization<T> serialization_;
  std::function<void(const RosbagReaderMsg<T> &)> callback_;

public:
  explicit SerializedMessageHandler(std::function<void(const RosbagReaderMsg<T> &)> callback)
  : callback_(callback)
  {
  }
  void handle_serialized_message(
    const rosbag2_storage::SerializedBagMessage & bag_msg,
    const rclcpp::SerializedMessage & ser_msg) override
  {
    T msg;
    serialization_.deserialize_message(&ser_msg, &msg);
    callback_({bag_msg, msg});
  }
};
class RosbagReader
{
public:
  // Constructor
  explicit RosbagReader(const std::string & bag_path) { reader_.open(bag_path); }
  /**
   * @brief Add a listener for a specific topic
   * @tparam T - ROS message type
   * @param topic_name - name of the topic
   * @param callback - callback function that will be called when a message is received
   */
  template <typename T>
  void add_listener(
    const std::string & topic_name, std::function<void(const RosbagReaderMsg<T> &)> callback)
  {
    handlers_[topic_name].push_back(std::make_unique<SerializedMessageHandler<T>>(callback));
  }
  /**
   * @brief Set the state flag to false to stop processing
   * @param state - flag to set
   */
  void set_state(bool state) { running_ = state; }
  /**
   * @brief Process the bag file
   */
  void process()
  {
    int64_t first_msg_time = -1;
    rclcpp::Clock wall_clock{};

    std::vector<std::string> topics{};
    topics.reserve(handlers_.size());
    for (auto & entry : handlers_) {
      topics.push_back(entry.first);
    }
    reader_.set_filter({topics});

    double total_duration = static_cast<double>((reader_.get_metadata().duration.count()) / 1.0e9);
    running_ = true;

    while (reader_.has_next() && rclcpp::ok() && running_) {
      auto bag_msg = reader_.read_next();
      rclcpp::SerializedMessage ser_msg{*bag_msg->serialized_data};

      if (first_msg_time < 0) {
        first_msg_time = bag_msg->time_stamp;
      }

      // Log progress
      double time_done = static_cast<double>((bag_msg->time_stamp - first_msg_time) / 1e9);
      RCLCPP_INFO_THROTTLE(
        logger_, wall_clock, 5000, "Processed %.3f sec of bag [% 5.1f%%]", time_done,
        100 * time_done / total_duration);

      auto it = handlers_.find(bag_msg->topic_name);
      if (it == handlers_.end()) {
        continue;
      }

      for (auto & handler : it->second) {
        handler->handle_serialized_message(*bag_msg, ser_msg);
      }
    }
  }

private:
  std::unordered_map<std::string, std::vector<std::shared_ptr<SerializedMessageHandlerBase>>>
    handlers_;
  bool running_{true};

  rclcpp::Logger logger_ = rclcpp::get_logger("rosbag_reader");
  rosbag2_cpp::Reader reader_;
};
template <typename T>
class BagSubscriber : public message_filters::SimpleFilter<T>
{
public:
  BagSubscriber(const std::string & topic_name, RosbagReader & reader, bool bag_time = false)
  {
    reader.add_listener<T>(topic_name, [this, bag_time](const RosbagReaderMsg<T> & msg) {
      auto msg_ptr = std::make_shared<T>(msg.ros_msg);

      if (bag_time) {
        msg_ptr->header.stamp.sec = static_cast<int>(msg.bag_msg.time_stamp / 1000000000L);
        msg_ptr->header.stamp.nanosec = msg.bag_msg.time_stamp % 1000000000L;
      }

      this->signalMessage(msg_ptr);
    });
  }
};
}  // namespace pointcloudcrafter::tools
