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
  XYZ,
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
    case FileFormat::XYZ: return ".xyz";
    case FileFormat::KITTI: return ".bin";
    case FileFormat::NUSCENES: return ".bin";
    case FileFormat::OBJ: return ".obj";
    default: return ".pcd";
  }
}
}  // namespace pointcloudcrafter::tools::formats
