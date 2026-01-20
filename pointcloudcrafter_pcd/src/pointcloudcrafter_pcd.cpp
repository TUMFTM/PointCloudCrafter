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

#include "pointcloudmodifier.hpp"

#include <fmt/core.h>

#include <algorithm>
#include <filesystem>  // NOLINT
#include <rclcpp/logging.hpp>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "pointcloudcrafter_pcd/pointcloudcrafter_pcd.hpp"

namespace pointcloudcrafter
{
PCD::PCD(const config::PCDConfig & cfg)
: logger_(rclcpp::get_logger("pointcloudcrafter_pcd")), cfg_(cfg)
{
  std::filesystem::path input_path(cfg_.input_path);
  if (std::filesystem::is_directory(input_path)) {
    for (const auto & entry : std::filesystem::directory_iterator(input_path)) {
      if (entry.path().extension() == ".pcd") {
        pcd_files_.push_back(entry.path().string());
      }
    }
    std::sort(pcd_files_.begin(), pcd_files_.end());
  } else if (std::filesystem::is_regular_file(input_path) && input_path.extension() == ".pcd") {
    pcd_files_.push_back(cfg_.input_path);
  } else {
    throw std::runtime_error("Input path must be a PCD file or directory containing PCD files");
  }

  if (pcd_files_.empty()) {
    throw std::runtime_error("No PCD files found in input path");
  }

  RCLCPP_INFO(logger_, "Found %zu PCD file(s) to process", pcd_files_.size());

  if (!std::filesystem::exists(cfg_.out_dir)) {
    std::filesystem::create_directories(cfg_.out_dir);
  }
}

void PCD::run()
{
  int64_t file_index = 0;
  rclcpp::Clock wall_clock{};
  const int64_t total_files = static_cast<int64_t>(pcd_files_.size());

  for (const auto & pcd_file : pcd_files_) {
    if (file_index < cfg_.skip_frames) {
      file_index++;
      continue;
    }

    if (stride_frames_ > 0) {
      stride_frames_--;
      file_index++;
      continue;
    }

    if (cfg_.max_frames > 0 && processed_frames_ >= cfg_.max_frames) {
      RCLCPP_INFO(logger_, "Reached max frames limit (%ld)", cfg_.max_frames);
      break;
    }

    // Log progress
    double progress = 100.0 * static_cast<double>(file_index) / static_cast<double>(total_files);
    RCLCPP_INFO_THROTTLE(logger_, wall_clock, 1000, "Processed %ld of %ld files [% 5.1f%%]",
      file_index, total_files, progress);

    process_pointcloud(pcd_file);

    processed_frames_++;
    stride_frames_ = cfg_.stride_frames - 1;
    file_index++;
  }

  RCLCPP_INFO(logger_, "Processing complete. Processed %ld file(s).", processed_frames_);
}

void PCD::process_pointcloud(const std::string & input_path)
{
  pointcloudmodifierlib::Modifier modifier;
  if (!modifier.loadPCD(input_path)) {
    RCLCPP_ERROR(logger_, "Failed to load PCD file: %s", input_path.c_str());
    return;
  }

  // Apply filters
  if (!cfg_.cropbox.empty()) {
    modifier.cropBox(cfg_.cropbox, cfg_.inverse_crop);
  }
  if (cfg_.cropsphere > 0.0) {
    modifier.cropSphere(cfg_.cropsphere, cfg_.inverse_crop);
  }
  if (cfg_.cropcylinder > 0.0) {
    modifier.cropCylinder(cfg_.cropcylinder, cfg_.inverse_crop);
  }
  if (!cfg_.voxelfilter.empty()) {
    modifier.voxelFilter(cfg_.voxelfilter);
  }
  if (cfg_.outlier_radius_filter.first > 0.0) {
    modifier.outlierRadiusFilter(cfg_.outlier_radius_filter.first,
      cfg_.outlier_radius_filter.second);
  }
  if (cfg_.outlier_stat_filter.first > 0.0) {
    modifier.outlierStatFilter(cfg_.outlier_stat_filter.first, cfg_.outlier_stat_filter.second);
  }

  std::string output_name;
  if (cfg_.sequential_names) {
    output_name = fmt::format("{:06d}.pcd", processed_frames_);
  } else {
    std::filesystem::path p(input_path);
    output_name = p.filename().string();
  }

  std::string output_path = cfg_.out_dir + "/" + output_name;
  if (!modifier.savePCD(output_path)) {
    RCLCPP_ERROR(logger_, "Failed to save pointcloud to %s", output_path.c_str());
    return;
  }
}

}  // namespace pointcloudcrafter
