/*
 * Copyright 2024 Markus Pielmeier, Florian Sauerbeck,
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

#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <utility>
#include <vector>

namespace pointcloudmodifier
{
// Flags - CLI arguments (defined in pcdmodifier.cpp)
extern std::string INPUT_PATH;
extern std::string OUT_DIR;
extern std::string TRANSFORM_FILE;
extern int64_t MAX_FRAMES;
extern int64_t SKIP_FRAMES;
extern int64_t STRIDE_FRAMES;
extern bool SEQUENTIAL_NAMES;
extern std::vector<double> CROPBOX;
extern double CROPSPHERE;
extern double CROPCYLINDER;
extern std::vector<double> VOXELFILTER;
extern std::pair<double, int> OUTLIERRADIUSFILTER;
extern std::pair<double, int> OUTLIERSTATFILTER;

/**
 * @brief PointCloudModifier class
 */
class PointCloudModifier
{
public:
  PointCloudModifier();
  void run();

protected:
  void process_pointcloud(const std::string & input_path);

private:
  rclcpp::Logger logger_;
  std::vector<std::string> pcd_files_{};
  int64_t processed_frames_{0};
  int64_t stride_frames_{0};
};

}  // namespace pointcloudmodifier
