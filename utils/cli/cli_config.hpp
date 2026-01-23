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
#include <utility>
#include <vector>

#include "cli/CLI11.hpp"
#include "formats.hpp"

namespace config
{
/**
 * @brief Common modifier configuration
 */
struct ModifierConfig
{
  std::string transform_file{};
  std::vector<double> cropbox{};
  double cropsphere{0.0};
  double cropcylinder{0.0};
  std::vector<double> voxelfilter{};
  std::pair<double, int> outlier_radius_filter{};
  std::pair<double, int> outlier_stat_filter{};
  int64_t max_frames{-1};
  int64_t skip_frames{0};
  int64_t stride_frames{1};
  std::string out_dir{};
  bool sequential_names{false};
  bool inverse_crop{false};
  bool load_pcd{true};
  bool load_ply{false};
  bool load_txt{false};
  bool load_kitti{false};
  bool load_nuscenes{false};
  bool load_obj{false};
  bool save_pcd{false};
  bool save_ply{false};
  bool save_txt{false};
  bool save_kitti{false};
  bool save_nuscenes{false};

  /**
   * @brief Get the load format based on flags
   * @return FileFormat enum value
   */
  pointcloudcrafter::tools::formats::FileFormat get_load_format() const
  {
    if (load_ply) return pointcloudcrafter::tools::formats::FileFormat::PLY;
    if (load_txt) return pointcloudcrafter::tools::formats::FileFormat::TXT;
    if (load_kitti) return pointcloudcrafter::tools::formats::FileFormat::KITTI;
    if (load_nuscenes) return pointcloudcrafter::tools::formats::FileFormat::NUSCENES;
    if (load_obj) return pointcloudcrafter::tools::formats::FileFormat::OBJ;
    return pointcloudcrafter::tools::formats::FileFormat::PCD;
  }

  /**
   * @brief Get the save format based on flags
   * @return FileFormat enum value (default: same as load format)
   */
  pointcloudcrafter::tools::formats::FileFormat get_save_format() const
  {
    if (save_pcd) return pointcloudcrafter::tools::formats::FileFormat::PCD;
    if (save_ply) return pointcloudcrafter::tools::formats::FileFormat::PLY;
    if (save_txt) return pointcloudcrafter::tools::formats::FileFormat::TXT;
    if (save_kitti) return pointcloudcrafter::tools::formats::FileFormat::KITTI;
    if (save_nuscenes) return pointcloudcrafter::tools::formats::FileFormat::NUSCENES;
    return get_load_format();
  }

  /**
   * @brief Add common modifier CLI options to the given app
   * @param app CLI app
   */
  void add_modifier_options(CLI::App * app)
  {
    // Output
    app->add_flag("--sequential-name", sequential_names, "Use sequential file names")
      ->group("Output");

    // Frame selection
    app->add_option("-m,--max-frames", max_frames, "Maximum number of frames (-1 = unlimited)")
      ->group("General");

    app->add_option("-j,--skip-frames", skip_frames, "Number of frames to skip at the beginning")
      ->group("General");

    app->add_option("-s,--stride-frames", stride_frames, "Write every Nth frame")
      ->group("General");

    // File format
    app->add_flag("--save-pcd", save_pcd, "Save PCD files (default)")
      ->group("File Format");
    app->add_flag("--save-ply", save_ply, "Save PLY files")
      ->group("File Format");
    app->add_flag("--save-txt", save_txt, "Save TXT ASCII files")
      ->group("File Format");
    app->add_flag("--save-kitti", save_kitti, "Save KITTI binary files")
      ->group("File Format");
    app->add_flag("--save-nuscenes", save_nuscenes, "Save nuScenes binary files")
      ->group("File Format");

    // Transforms
    app->add_option(
        "--tf, --transform-file", transform_file,
        "TXT file with transform ([frame_id] r1 r2 r3 x r4 r5 r6 y r7 r8 r9 z)")
      ->group("Transforms");

    // Filtering
    app->add_option("--cb, --crop-box", cropbox, "Crop box [xmin ymin zmin xmax ymax zmax]")
      ->expected(6)
      ->type_name("FLOAT FLOAT FLOAT FLOAT FLOAT FLOAT")
      ->group("Filtering");

    app->add_option("--cs, --crop-sphere", cropsphere, "Crop to sphere with given radius")
      ->type_name("FLOAT")
      ->group("Filtering");

    app->add_option("--cc, --crop-cylinder", cropcylinder, "Crop to cylinder with given radius")
      ->type_name("FLOAT")
      ->group("Filtering");

    app->add_flag("--inverse-crop", inverse_crop, "Inverse crop filters")
      ->group("Filtering");

    app->add_option("--vf, --voxel-filter", voxelfilter, "Voxel size [x y z]")
      ->expected(3)
      ->type_name("FLOAT FLOAT FLOAT")
      ->group("Filtering");

    app->add_option(
        "--orf, --outlier-radius-filter", outlier_radius_filter,
        "Radius outlier removal [radius min_neighbors]")
      ->expected(1)
      ->type_name("FLOAT INT")
      ->group("Filtering");

    app->add_option(
        "--osf, --outlier-stat-filter", outlier_stat_filter,
        "Statistical outlier removal [threshold mean_k]")
      ->expected(1)
      ->type_name("FLOAT INT")
      ->group("Filtering");
  }
};

/**
 * @brief PCFile input configuration
 */
struct FileConfig : public ModifierConfig
{
  std::string input_path{};
  std::vector<double> translation{};
  std::vector<double> rotation{};
  bool degree{false};

  /**
   * @brief Add PCFile CLI options to the given app
   * @param app CLI app
   */
  void add_cli_options(CLI::App * app)
  {
    app->add_option("input-path", input_path, "Path to point cloud file or directory")
      ->required()
      ->group("Required");

    app->add_option("out-dir", out_dir, "Output directory for point cloud file(s)")
      ->required()
      ->group("Required");

    app->add_flag("--load-pcd", load_pcd, "Load PCD files (default)")
      ->group("File Format");
    app->add_flag("--load-ply", load_ply, "Load PLY files")
      ->group("File Format");
    app->add_flag("--load-txt", load_txt, "Load TXT ASCII files")
      ->group("File Format");
    app->add_flag("--load-kitti", load_kitti, "Load KITTI binary files")
      ->group("File Format");
    app->add_flag("--load-nuscenes", load_nuscenes, "Load nuScenes binary files")
      ->group("File Format");
    app->add_flag("--load-obj", load_obj, "Load OBJ files")
      ->group("File Format");

    app->add_option("-t,--translation", translation, "Translation [x y z]")
      ->expected(3)
      ->type_name("FLOAT FLOAT FLOAT")
      ->group("Transforms");

    app->add_option("-r,--rotation", rotation, "Rotation [roll pitch yaw]")
      ->expected(3)
      ->type_name("FLOAT FLOAT FLOAT")
      ->group("Transforms");

    app->add_flag("--deg", degree, "Rotation in degrees instead of radians")
      ->group("Transforms");

    add_modifier_options(app);
  }
};

/**
 * @brief Rosbag input configuration
 */
struct RosbagConfig : public ModifierConfig
{
  std::string bag_path{};
  std::vector<std::string> topics{};
  std::string target_frame{};
  bool timestamps{false};

  /**
   * @brief Add Rosbag CLI options to the given app
   * @param app CLI app
   */
  void add_cli_options(CLI::App * app)
  {
    app->add_option("bag-path", bag_path, "Path to ROS 2 bag")
      ->required()
      ->group("Required");

    app->add_option("out-dir", out_dir, "Output directory for point cloud file(s)")
      ->required()
      ->group("Required");

    app->add_option("topic-names", topics, "PointCloud2 topic names")
      ->required()
      ->group("Required");

    app->add_flag("--timestamps", timestamps, "Save point cloud timestamps to a text file")
      ->group("Output");

    app->add_option("-t,--target-frame", target_frame, "Target TF frame for all point clouds")
      ->group("Transforms");

    add_modifier_options(app);
  }
};

}  // namespace config
