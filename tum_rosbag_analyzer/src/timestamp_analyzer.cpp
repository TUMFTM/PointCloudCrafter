// Copyright 2024 Maximilian Leitenstern
#include "tum_rosbag_analyzer/timestamp_analyzer.hpp"

int analyze_timestamps(std::string input_path, std::string topic_name)
{
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
  std::unique_ptr<rosbag2_cpp::Reader> reader = std::make_unique<rosbag2_cpp::Reader>();

  std::vector<double> bag_diff{};

  reader->open(input_path);
  while (reader->has_next()) {
    rosbag2_storage::SerializedBagMessageSharedPtr msg = reader->read_next();

    // Handle pointclouds
    if (msg->topic_name == topic_name) {
      // Deserialize bag message
      rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
      auto ros_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
      serialization.deserialize_message(&serialized_msg, ros_msg.get());

      // Extract timestamps
      const auto msg_stamp = rclcpp::Time(ros_msg->header.stamp).nanoseconds();
      std::vector<double> stamps = GetTimestamps(*ros_msg);
      double diff =
        *max_element(stamps.begin(), stamps.end()) - *min_element(stamps.begin(), stamps.end());
      std::cout << "PointCloud" << std::endl;
      std::cout << "size: " << stamps.size() << std::endl;
      std::cout << std::fixed << std::setprecision(9);
      std::cout << "msg_stamp: " << msg_stamp * (1e-9) << std::endl;
      std::cout << "max_stamp: " << *max_element(stamps.begin(), stamps.end()) * (1e-9) << std::endl
                << "min_stamp: " << *min_element(stamps.begin(), stamps.end()) * (1e-9)
                << std::endl;
      bag_diff.push_back(diff);
    }
  }
  // std::cout << std::setprecision(15)
  //           << "Max time difference between timestamps: " << *max_element(bag_diff.begin(),
  //           bag_diff.end())
  //           << std::endl
  //           << "Min time difference between timestamps: " << *min_element(bag_diff.begin(),
  //           bag_diff.end())
  //           << std::endl
  //           << "Mean time difference between timestamps: "
  //           << std::accumulate(bag_diff.begin(), bag_diff.end(), 0.0) /
  //           static_cast<double>(bag_diff.size())
  //           << std::endl;
  return 0;
}
std::vector<double> GetTimestamps(const sensor_msgs::msg::PointCloud2 & msg)
{
  sensor_msgs::msg::PointField timestamp_field = GetTimestampField(msg);

  // Extract timestamps from cloud_msg
  std::vector<double> timestamps = ExtractTimestampsFromMsg(msg, timestamp_field);

  return timestamps;
}
sensor_msgs::msg::PointField GetTimestampField(const sensor_msgs::msg::PointCloud2 & msg)
{
  sensor_msgs::msg::PointField timestamp_field;
  for (const auto & field : msg.fields) {
    if (field.name == "timestamp" || field.name == "time_stamp" || field.name == "t") {
      timestamp_field = field;
    }
  }
  if (!timestamp_field.count) {
    throw std::runtime_error("Field 'timestamp'  does not exist");
  }
  return timestamp_field;
}
std::vector<double> ExtractTimestampsFromMsg(
  const sensor_msgs::msg::PointCloud2 & msg, const sensor_msgs::msg::PointField & field)
{
  // Extract timestamps from cloud_msg
  const size_t n_points = msg.height * msg.width;
  std::vector<double> timestamps;
  timestamps.reserve(n_points);

  // Timestamps are doubles -> time in sec as offset to center time
  if (field.name == "timestamp" || field.name == "time_stamp" || field.name == "t") {
    if (field.datatype == pcl::PCLPointField::FLOAT32) {  // type 7
      sensor_msgs::PointCloud2ConstIterator<float> msg_t(msg, field.name);
      for (size_t i = 0; i < n_points; ++i, ++msg_t) {
        timestamps.emplace_back(*msg_t);
      }
    } else if (field.datatype == pcl::PCLPointField::FLOAT64) {  // type 8
      sensor_msgs::PointCloud2ConstIterator<double> msg_t(msg, field.name);
      for (size_t i = 0; i < n_points; ++i, ++msg_t) {
        timestamps.emplace_back(*msg_t);
      }
    } else if (field.datatype == pcl::PCLPointField::UINT32) {  // type 6
      sensor_msgs::PointCloud2ConstIterator<uint32_t> msg_t(msg, field.name);
      for (size_t i = 0; i < n_points; ++i, ++msg_t) {
        timestamps.emplace_back(*msg_t);
      }
    } else if (field.datatype == pcl::PCLPointField::UINT8) {  // type 2 - array of 8 uint8 (luminar
                                                               // iris driver)
      sensor_msgs::PointCloud2ConstIterator<uint8_t> msg_t(msg, field.name);
      for (size_t i = 0; i < n_points; ++i, ++msg_t) {
        uint64_t stamp;
        std::memcpy(&stamp, &*msg_t, sizeof(uint64_t));
        timestamps.emplace_back(static_cast<double>(stamp));
      }
    } else {
      std::cout << "Time field of type != 2,6,7,8" << std::endl;
      exit(EXIT_FAILURE);
    }
  }
  return timestamps;
}
