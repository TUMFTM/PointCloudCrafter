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

#include "CLI11.hpp"

namespace config
{

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

    // Transforms
    app->add_option(
        "--transform-file,--tf", transform_file,
        "TXT file with transform (r1 r2 r3 x r4 r5 r6 y r7 r8 r9 z)")
      ->group("Transforms");

    // Filtering
    app->add_option("--crop-box,--cb", cropbox, "Crop box [xmin ymin zmin xmax ymax zmax]")
      ->expected(6)
      ->type_name("FLOAT FLOAT FLOAT FLOAT FLOAT FLOAT")
      ->group("Filtering");

    app->add_option("--crop-sphere,--cs", cropsphere, "Crop to sphere with given radius")
      ->type_name("FLOAT")
      ->group("Filtering");

    app->add_option("--crop-cylinder,--cc", cropcylinder, "Crop to cylinder with given radius")
      ->type_name("FLOAT")
      ->group("Filtering");

    app->add_option("--voxel-filter,--vf", voxelfilter, "Voxel size [x y z]")
      ->expected(3)
      ->type_name("FLOAT FLOAT FLOAT")
      ->group("Filtering");

    app->add_option(
        "--outlier-radius-filter,--orf", outlier_radius_filter,
        "Radius outlier removal [radius min_neighbors]")
      ->expected(2)
      ->type_name("FLOAT INT")
      ->group("Filtering");

    app->add_option(
        "--outlier-stat-filter,--osf", outlier_stat_filter,
        "Statistical outlier removal [threshold mean_k]")
      ->expected(2)
      ->type_name("FLOAT INT")
      ->group("Filtering");
  }
};

struct PcdModifierConfig : public ModifierConfig
{
  std::string input_path{};

  void add_cli_options(CLI::App * app)
  {
    app->add_option("input-path", input_path, "Path to PCD file or directory")
      ->required()
      ->group("Required");

    app->add_option("out-dir", out_dir, "Output directory for .pcd files")
      ->required()
      ->group("Required");

    add_modifier_options(app);
  }
};

struct CrafterConfig : public ModifierConfig
{
  std::string bag_path{};
  std::vector<std::string> topics{};
  std::string target_frame{};
  bool timestamps{false};

  void add_cli_options(CLI::App * app)
  {
    app->add_option("bag-path", bag_path, "Path to ROS 2 bag")
      ->required()
      ->group("Required");

    app->add_option("out-dir", out_dir, "Output directory for .pcd files")
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
