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

#include "pointcloudcrafter_file/pointcloudcrafter_file.hpp"
#include "pointcloudmodifier.hpp"
#include "utils.hpp"

#include <fmt/core.h>

#include <algorithm>
#include <filesystem>  // NOLINT
#include <rclcpp/logging.hpp>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace pointcloudcrafter
{
/**
 * @brief PCFile class
 * @param cfg Configuration
 */
PCFile::PCFile(const config::FileConfig & cfg)
: cfg_(cfg),
  logger_(rclcpp::get_logger("pointcloudcrafter_file"))
{
  std::filesystem::path input_path(cfg_.input_path);
  auto load_fmt = cfg_.get_load_format();

  if (std::filesystem::is_directory(input_path)) {
    for (const auto & entry : std::filesystem::directory_iterator(input_path)) {
      if (pointcloudcrafter::tools::formats::matches_format(entry.path(), load_fmt)) {
        pc_files_.push_back(entry.path().string());
      }
    }
    std::sort(pc_files_.begin(), pc_files_.end());
  } else if (std::filesystem::is_regular_file(input_path) &&
             pointcloudcrafter::tools::formats::matches_format(input_path, load_fmt)) {
    pc_files_.push_back(cfg_.input_path);
  } else {
    throw std::runtime_error(
      fmt::format("Input path must be a '{}' file or directory containing '{}' files",
        pointcloudcrafter::tools::formats::format_to_extension(load_fmt),
        pointcloudcrafter::tools::formats::format_to_extension(load_fmt))
    );
  }

  if (pc_files_.empty()) {
    throw std::runtime_error("No point cloud files found in input path");
  }

  RCLCPP_INFO(logger_, "Found %zu point cloud file(s) to process", pc_files_.size());

  // Load transforms from file
  if (!cfg_.transform_file.empty()) {
    this->file_transforms_ = tools::utils::load_poses_from_file(cfg_.transform_file);
  }

  // Init out directory
  if (!std::filesystem::exists(cfg_.out_dir)) {
    std::filesystem::create_directories(cfg_.out_dir);
  }
}

/**
 * @brief Run the point cloud file processing
 */
void PCFile::run()
{
  bool last_file = false;
  int64_t file_index = 0;
  rclcpp::Clock wall_clock{};
  const int64_t total_files = static_cast<int64_t>(pc_files_.size());

  for (size_t i = 0; i < total_files && rclcpp::ok(); i++) {
    // Check if we should skip frames
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

    if (file_index == total_files - 1 || processed_frames_ == cfg_.max_frames - 1) {
      last_file = true;
    }
    process_pointcloud(pc_files_[i], file_index, last_file);

    processed_frames_++;
    stride_frames_ = cfg_.stride_frames - 1;
    file_index++;
  }

  RCLCPP_INFO(logger_, "Processing finished. Processed %ld file(s).", processed_frames_);
}

/**
 * @brief Process a single pointcloud file
 * @param input_path Path to the input point cloud file
 * @param file_index Index of the file being processed
 * @param last_file Whether this is the last file to be processed
 */
void PCFile::process_pointcloud(const std::string & input_path,
  const size_t & file_index, const bool & last_file)
{
  // Modify the pointclouds with pointcloudmodifierlib
  pointcloudmodifierlib::Modifier modifier;
  if (!modifier.load(input_path, cfg_.get_load_format())) {
    RCLCPP_ERROR(logger_, "Failed to load file: %s", input_path.c_str());
    return;
  }

  // Apply filters
  // Cropbox filtering
  if (!cfg_.cropbox.empty()) {
    modifier.cropBox(cfg_.cropbox, cfg_.inverse_crop);
  }
  // Sphere filtering
  if (cfg_.cropsphere > 0.0) {
    modifier.cropSphere(cfg_.cropsphere, cfg_.inverse_crop);
  }
  // Cylinder filtering
  if (cfg_.cropcylinder > 0.0) {
    modifier.cropCylinder(cfg_.cropcylinder, cfg_.inverse_crop);
  }
  // Voxelization
  if (!cfg_.voxelfilter.empty()) {
    modifier.voxelFilter(cfg_.voxelfilter);
  }
  // Outlier radius filtering
  if (cfg_.outlier_radius_filter.first > 0.0) {
    modifier.outlierRadiusFilter(cfg_.outlier_radius_filter.first,
      cfg_.outlier_radius_filter.second);
  }
  // Outlier statistical filtering
  if (cfg_.outlier_stat_filter.first > 0.0) {
    modifier.outlierStatFilter(cfg_.outlier_stat_filter.first, cfg_.outlier_stat_filter.second);
  }
  // Apply transforms
  // Pose file transform
  if (!cfg_.transform_file.empty() && file_index < file_transforms_.size()) {
    modifier.transform(this->file_transforms_[file_index]);
  } else if (!cfg_.transform_file.empty()) {
    RCLCPP_ERROR(logger_, "No transform available for index %zu", file_index);
    return;
  }
  // Rotation transform
  if (!cfg_.rotation.empty()) {
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    std::vector<double> rot =
        cfg_.degree ? std::vector<double>{
          cfg_.rotation[0] * M_PI / 180.0,
          cfg_.rotation[1] * M_PI / 180.0,
          cfg_.rotation[2] * M_PI / 180.0}
            : cfg_.rotation;
    Eigen::Matrix3d rotation;
    rotation = Eigen::AngleAxisd(rot[0], Eigen::Vector3d::UnitX()) *
               Eigen::AngleAxisd(rot[1], Eigen::Vector3d::UnitY()) *
               Eigen::AngleAxisd(rot[2], Eigen::Vector3d::UnitZ());
    transform.linear() = rotation;
    modifier.transform(transform);
  }
  // Translation transform
  if (!cfg_.translation.empty()) {
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.translation() << cfg_.translation[0], cfg_.translation[1], cfg_.translation[2];
    modifier.transform(transform);
  }

  // Merge clouds
  std::string stem{};
  if (cfg_.merge_clouds) {
    *merged_cloud += *modifier.getOutputCloud();
    if (last_file) {
      modifier.setCloud(merged_cloud);
      stem = "merged_cloud";
    } else {
    return;
    }
  } else {
    std::filesystem::path input_p(input_path);
    stem = cfg_.sequential_names ?
    fmt::format("{:06d}", processed_frames_) : pointcloudcrafter::tools::formats::get_stem(input_p);
  }

  // Generate output path
  auto save_fmt = cfg_.get_save_format();
  std::string output_path = cfg_.out_dir + "/" + stem +
    pointcloudcrafter::tools::formats::format_to_extension(save_fmt);

  // Save with configured format
  if (!modifier.save(output_path, save_fmt)) {
    RCLCPP_ERROR(logger_, "Failed to save: %s", output_path.c_str());
  }

  // Save timestamps if enabled
  if (cfg_.timestamps) {
    modifier.timestampAnalyzer(cfg_.out_dir + "/" + stem + "_stamps.txt");
  }
}

}  // namespace pointcloudcrafter
