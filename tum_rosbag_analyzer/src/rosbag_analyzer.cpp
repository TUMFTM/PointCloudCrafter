// Copyright 2024 Maximilian Leitenstern

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "tum_rosbag_analyzer/rosbag_analyzer.hpp"

#include <iostream>
#include <string>
int main(int argc, char ** argv)
{
  // Get rosbag analyzer mode
  std::string mode = std::string(argv[1]);

  // Sanity check for mode
  if (mode != "timestamps") {
    std::cout << "Usage: ros2 run rosbag_analyzer rosbag_analyzer <mode> <other options ...>"
              << std::endl;
    std::cout << "Mode is either \"timestamps\" or \"tbd\"!" << std::endl;
    return -1;
  }

  // Sanity check for amount of arguments
  if (mode == "timestamps") {
    if (argc != 4) {
      std::cout
        << "Usage: ros2 run rosbag_analyzer rosbag_analyzer timestamps <input path> <pcl_topic>"
        << std::endl;
      std::cout << "Input path should be a rosbag file or directory!" << std::endl;
      std::cout << "" << std::endl;
      return -1;
    }

    // Get input arguments
    std::string input_path = std::string(argv[2]);
    std::string topic = std::string(argv[3]);

    // Process rosbag
    auto ret = analyze_timestamps(input_path, topic);
    if (ret == 0) {
      std::cout << "Successfully processed Rosbag!" << std::endl;
    } else {
      std::cout << "Failed to process Rosbag!" << std::endl;
    }
    return ret;
  }
}
