/*
 * Copyright 2024 2024 Markus Pielmeier, Florian Sauerbeck,
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

#include <rclcpp/rclcpp.hpp>

#include "CLI11.hpp"
#include "pointcloudcrafter/pointcloudcrafter.hpp"
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  CLI::App app{"rosbag_to_pcd"};
  app.option_defaults()->always_capture_default();
  app.add_option("bag-path", pointcloudcrafter::BAG_PATH, "Path to ROS2 Bag")->required();
  app.add_option("out-dir", pointcloudcrafter::OUT_DIR, "Output directory for .pcd files")
    ->required();
  app.add_option("topic-names", pointcloudcrafter::TOPICS, "Name of ROS topic of type PointCloud2")
    ->required();
  app.add_option(
    "--target-frame, -t", pointcloudcrafter::TARGET_FRAME,
    "Target TF2 frame which all pointclouds are transformed to.");
  app.add_option(
    "--max-frames, -m", pointcloudcrafter::MAX_FRAMES,
    "Maximum number of frames to extract. Use -1 for no limit");
  app.add_option(
    "--skip-frames, -j", pointcloudcrafter::SKIP_FRAMES,
    "Number of frames to skip at the beginning");
  app.add_option(
    "--stride-frames, -s", pointcloudcrafter::STRIDE_FRAMES, "Write every Nth frame to file");
  app.add_option(
    "--transform-file, --tf", pointcloudcrafter::TRANSFORM_FILE,
    "Path to yaml file with extra transforms");
  app.add_flag(
    "--sequential-name", pointcloudcrafter::SEQUENTIAL_NAMES,
    "Name files sequentially instead of based on their timestamp");
  app.add_option(
    "--crop-box, --cb", pointcloudcrafter::CROPBOX,
    "Set crop box parmas around orgin [x_min y_min z_min x_max y_max z_max]");
  app.add_option(
    "--crop-sphere, --cs", pointcloudcrafter::CROPSPHERE,
    "Crop clouds to sphere with a given radius r");
  app.add_option(
    "--crop-cylinder, --cc", pointcloudcrafter::CROPCYLINDER,
    "Crop clouds to cylinder with a given radius r");
  app.add_option(
    "--voxel-filter, --vf", pointcloudcrafter::VOXELFILTER,
    "Voxelize clouds with [voxel_size_x, voxel_size_y, voxel_size_z]");
  app.add_option(
    "--outlier-radius-filter, --orf", pointcloudcrafter::OUTLIERRADIUSFILTER,
    "Remove outliers with a given radius and minimum number of neighbors [radius, min_neighbors]"
  );
  app.add_option(
    "--outlier-stat-filter, --osf", pointcloudcrafter::OUTLIERSTATFILTER,
    "Remove outliers with a given threshold and mean [threshold, mean]");

  CLI11_PARSE(app, argc, argv);

  pointcloudcrafter::PointCloudCrafter().run();
  rclcpp::shutdown();
  return 0;
}
