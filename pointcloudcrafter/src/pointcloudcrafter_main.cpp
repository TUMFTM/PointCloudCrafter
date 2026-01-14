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

#include <rclcpp/rclcpp.hpp>

#include "CLI11.hpp"
#include "pointcloudcrafter/pointcloudcrafter.hpp"
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  CLI::App app{"rosbag_to_pcd"};
  app.name("ros2 run pointcloudcrafter crafter");

  /* =========================
   * Positional arguments
   * ========================= */
  app.add_option("bag-path", pointcloudcrafter::BAG_PATH, "Path to ROS 2 bag")
    ->required()
    ->group("Required");

  app.add_option("out-dir", pointcloudcrafter::OUT_DIR, "Output directory for .pcd files")
    ->required()
    ->group("Required");

  app.add_option("topic-names", pointcloudcrafter::TOPICS, "PointCloud2 topic names")
    ->required()
    ->group("Required");

  /* =========================
   * Output control
   * ========================= */
  app
    .add_flag(
      "--sequential-name", pointcloudcrafter::SEQUENTIAL_NAMES,
      "Use sequential file names instead of timestamps")
    ->group("Output");

  app
    .add_flag(
      "--timestamps", pointcloudcrafter::TIMESTAMPS, "Save point cloud timestamps to a text file")
    ->group("Output");

  /* =========================
   * Frame selection
   * ========================= */
  app
    .add_option(
      "-m,--max-frames", pointcloudcrafter::MAX_FRAMES,
      "Maximum number of frames to extract (-1 = unlimited)")
    ->group("General");

  app
    .add_option(
      "-j,--skip-frames", pointcloudcrafter::SKIP_FRAMES,
      "Number of frames to skip at the beginning")
    ->group("General");

  app.add_option("-s,--stride-frames", pointcloudcrafter::STRIDE_FRAMES, "Write every Nth frame")
    ->group("General");

  /* =========================
   * Transform options
   * ========================= */
  app
    .add_option(
      "-t,--target-frame", pointcloudcrafter::TARGET_FRAME, "Target TF frame for all point clouds")
    ->group("Transforms");

  app
    .add_option(
      "--transform-file,--tf", pointcloudcrafter::TRANSFORM_FILE,
      "YAML file with additional transforms")
    ->group("Transforms");

  /* =========================
   * Filtering
   * ========================= */
  app
    .add_option(
      "--crop-box,--cb", pointcloudcrafter::CROPBOX, "Crop box [xmin ymin zmin xmax ymax zmax]")
    ->expected(6)
    ->type_name("FLOAT FLOAT FLOAT FLOAT FLOAT FLOAT")
    ->group("Filtering");

  app
    .add_option(
      "--crop-sphere,--cs", pointcloudcrafter::CROPSPHERE, "Crop to sphere with given radius")
    ->type_name("FLOAT")
    ->group("Filtering");

  app
    .add_option(
      "--crop-cylinder,--cc", pointcloudcrafter::CROPCYLINDER, "Crop to cylinder with given radius")
    ->type_name("FLOAT")
    ->group("Filtering");

  app.add_option("--voxel-filter,--vf", pointcloudcrafter::VOXELFILTER, "Voxel size [x y z]")
    ->expected(3)
    ->type_name("FLOAT FLOAT FLOAT")
    ->group("Filtering");

  app
    .add_option(
      "--outlier-radius-filter,--orf", pointcloudcrafter::OUTLIERRADIUSFILTER,
      "Radius outlier removal [radius min_neighbors]")
    ->expected(2)
    ->type_name("FLOAT INT")
    ->group("Filtering");

  app
    .add_option(
      "--outlier-stat-filter,--osf", pointcloudcrafter::OUTLIERSTATFILTER,
      "Statistical outlier removal [threshold mean_k]")
    ->expected(2)
    ->type_name("FLOAT INT")
    ->group("Filtering");

  app.footer(
    "\nExample:\n"
    "  ros2 run pointcloudcrafter crafter bag.mcap out/ /points_raw \n"
    "    --voxel-filter 0.1 0.1 0.1 --stride-frames 5\n");

  CLI11_PARSE(app, argc, argv);

  pointcloudcrafter::PointCloudCrafter().run();

  rclcpp::shutdown();
  return 0;
}
