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

#include <pcl/PCLPointCloud2.h>

#include <string>
#include <utility>
#include <vector>
#include <unordered_map>
#include <Eigen/Eigen>

#include "cli/cli_config.hpp"

namespace pointcloudcrafter
{
/**
 * @brief PCFile class
 */
class PCFile
{
public:
  explicit PCFile(const config::FileConfig & cfg);
  void run();

protected:
  void process_pointcloud(const std::string & input_path,
    const size_t & file_index, const bool & last_file);

private:
  config::FileConfig cfg_;
  rclcpp::Logger logger_;
  std::vector<std::string> pc_files_{};
  int64_t processed_frames_{0};
  int64_t stride_frames_{0};
  std::vector<Eigen::Affine3d> file_transforms_{};
  pcl::PCLPointCloud2::Ptr merged_cloud{new pcl::PCLPointCloud2()};
};

}  // namespace pointcloudcrafter
