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
/**
 * @brief Struct to hold a ROS message and its corresponding rosbag message
 * @tparam T - ROS message type
 * @param bag_msg - rosbag message
 * @param ros_msg - ROS message
 */
template <typename T>
struct RosbagReaderMsg
{
  const rosbag2_storage::SerializedBagMessage & bag_msg;
  const T & ros_msg;
};
/**
 * @brief Base class for message handling
 */
class SerializedMessageHandlerBase
{
public:
  virtual void handle_serialized_message(
    const rosbag2_storage::SerializedBagMessage & bag_msg,
    const rclcpp::SerializedMessage & ser_msg) = 0;
};
/**
 * @brief Class to handle serialized messages with a callback
 * @tparam T - ROS message type
 */
template <typename T>
class SerializedMessageHandler : public SerializedMessageHandlerBase
{
public:
  /**
   * @brief Constructor from a callback function
   * @param callback - callback function
   */
  explicit SerializedMessageHandler(std::function<void(const RosbagReaderMsg<T> &)> callback)
  : callback_(callback)
  {
  }
  /**
   * @brief Deserialize the message and call the callback
   * @param bag_msg - rosbag message
   * @param ser_msg - serialized message
   */
  void handle_serialized_message(
    const rosbag2_storage::SerializedBagMessage & bag_msg,
    const rclcpp::SerializedMessage & ser_msg) override
  {
    T msg;
    serialization_.deserialize_message(&ser_msg, &msg);
    callback_({bag_msg, msg});
  }

private:
  rclcpp::Serialization<T> serialization_;
  std::function<void(const RosbagReaderMsg<T> &)> callback_;
};
class RosbagReader
{
public:
  /**
   * @brief Constructor from a bag file path
   * @param BAG_PATH - path to the bag file
   */
  explicit RosbagReader(const std::string & BAG_PATH) { reader_.open(BAG_PATH); }
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
   * @brief -> to be called after all listeners are added
   */
  void process()
  {
    int64_t first_msg_time = -1;
    rclcpp::Clock wall_clock{};

    // Configure the reader to only read the topics we are interested in
    std::vector<std::string> topics{};
    topics.reserve(handlers_.size());
    for (auto & entry : handlers_) {
      topics.push_back(entry.first);
    }
    reader_.set_filter({topics});

    // Initialize progress logging
    double total_duration = static_cast<double>((reader_.get_metadata().duration.count()) / 1.0e9);
    running_ = true;

    // Process the bag files
    while (reader_.has_next() && rclcpp::ok() && running_) {
      auto bag_msg = reader_.read_next();
      rclcpp::SerializedMessage ser_msg{*bag_msg->serialized_data};

      // Get the first message time
      if (first_msg_time < 0) {
        first_msg_time = bag_msg->time_stamp;
      }

      // Log progress
      double time_done = static_cast<double>((bag_msg->time_stamp - first_msg_time) / 1e9);
      RCLCPP_INFO_THROTTLE(
        logger_, wall_clock, 5000, "Processed %.3f sec of bag [% 5.1f%%]", time_done,
        100 * time_done / total_duration);

      // Call the handlers
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
  // Map to store the handlers for each topic
  std::unordered_map<std::string, std::vector<std::shared_ptr<SerializedMessageHandlerBase>>>
    handlers_;
  // Logger to print progress
  rclcpp::Logger logger_ = rclcpp::get_logger("rosbag_reader");
  // Reader to read the bag file
  rosbag2_cpp::Reader reader_;
  // State flag
  bool running_{true};
};
/**
 * @brief Class to subscribe to a topic and call a callback function
 * @tparam T - ROS message type
 * @param topic_name - name of the topic
 * @param reader - rosbag reader
 */
template <typename T>
class MsgFilter : public message_filters::SimpleFilter<T>
{
public:
  MsgFilter(const std::string & topic_name, RosbagReader & reader)
  {
    reader.add_listener<T>(topic_name, [this](const RosbagReaderMsg<T> & msg) {
      auto msg_ptr = std::make_shared<T>(msg.ros_msg);
      this->signalMessage(msg_ptr);
    });
  }
};
}  // namespace pointcloudcrafter::tools
