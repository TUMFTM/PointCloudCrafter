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

#include <string>

namespace pointcloudcrafter::tools::formats
{
/**
 * @brief Supported point cloud file formats
 */
enum class FileFormat
{
  PCD,
  PLY,
  TXT,
  KITTI,
  NUSCENES,
  OBJ
};
/**
 * @brief Get file extension for a format
 * @param format The file format
 * @return File extension as string
 */
inline std::string format_to_extension(FileFormat format)
{
  switch (format) {
    case FileFormat::PCD: return ".pcd";
    case FileFormat::PLY: return ".ply";
    case FileFormat::TXT: return ".txt";
    case FileFormat::KITTI: return ".bin";
    case FileFormat::NUSCENES: return ".pcd.bin";
    case FileFormat::OBJ: return ".obj";
    default: return ".pcd";
  }
}
/**
 * @brief Check if a file path matches a given format
 * 
 * Handles special case for KITTI vs nuScenes:
 * - .bin (KITTI)
 * - .pcd.bin (nuScenes)
 * 
 * @param path The filesystem path
 * @param format The expected format
 * @return True if the file extension matches the format
 */
inline bool matches_format(const std::filesystem::path & path, FileFormat format)
{
  std::string ext = path.extension().string();

  if (format == FileFormat::NUSCENES) {
    // Check for .pcd.bin
    if (ext == ".bin") {
      std::string stem = path.stem().string();
      return stem.size() > 4 && stem.substr(stem.size() - 4) == ".pcd";
    }
    return false;
  }

  if (format == FileFormat::KITTI) {
    // .bin but NOT .pcd.bin
    if (ext == ".bin") {
      std::string stem = path.stem().string();
      return stem.size() <= 4 || stem.substr(stem.size() - 4) != ".pcd";
    }
    return false;
  }

  return ext == format_to_extension(format);
}
/**
 * @brief Get the stem of a filename, handling .pcd.bin extension
 * @param path The filesystem path
 * @return The stem (filename without extension)
 */
inline std::string get_stem(const std::filesystem::path & path)
{
  std::string ext = path.extension().string();
  std::string stem = path.stem().string();

  // Handle .pcd.bin case (nuscenes)
  if (ext == ".bin" && stem.size() > 4 && stem.substr(stem.size() - 4) == ".pcd") {
    return stem.substr(0, stem.size() - 4);
  }

  return stem;
}
}  // namespace pointcloudcrafter::tools::formats
