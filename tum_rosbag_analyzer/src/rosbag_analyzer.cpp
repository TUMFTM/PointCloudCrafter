// Copyright 2024 Maximilian Leitenstern
#include "tum_rosbag_analyzer/rosbag_analyzer.hpp"

int main(int argc, char ** argv)
{
  std::string mode = std::string(argv[1]);

  if (mode != "timestamps") {
    std::cout << "Usage: ros2 run rosbag_analyzer rosbag_analyzer <mode> <other options ...>"
              << std::endl;
    std::cout << "Mode is either \"timestamps\" or \"tbd\"!" << std::endl;
    return -1;
  }

  if (mode == "timestamps") {
    if (argc != 4) {
      std::cout
        << "Usage: ros2 run rosbag_analyzer rosbag_analyzer timestamps <input path> <pcl_topic>"
        << std::endl;
      std::cout << "Input path should be a rosbag file or directory!" << std::endl;
      std::cout << "" << std::endl;
      return -1;
    }

    std::string input_path = std::string(argv[2]);
    std::string pcl_topic = std::string(argv[3]);

    auto ret = analyze_timestamps(input_path, pcl_topic);
    if (ret == 0) {
      std::cout << "Successfully processed Rosbag!" << std::endl;
    } else {
      std::cout << "Failed to process Rosbag!" << std::endl;
    }
    return ret;
  }
}
