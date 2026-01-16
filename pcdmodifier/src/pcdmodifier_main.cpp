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
#include "pcdmodifier.hpp"
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  CLI::App app{"pcd_modifier"};
  app.name("ros2 run pointcloudmodifier modifier");

  /* =========================
   * Positional arguments
   * ========================= */
  app.add_option("input-path", pointcloudmodifier::INPUT_PATH, "Path to PCD file")
    ->required()
    ->group("Required");

  app.add_option("out-dir", pointcloudmodifier::OUT_DIR, "Output directory for .pcd files")
    ->required()
    ->group("Required");

  /* =========================
   * Output control
   * ========================= */
  app
    .add_flag(
      "--sequential-name", pointcloudmodifier::SEQUENTIAL_NAMES,
      "Use sequential file names instead of input file names")
    ->group("Output");

  /* =========================
   * Frame selection
   * ========================= */
  app
    .add_option(
      "-m,--max-frames", pointcloudmodifier::MAX_FRAMES,
      "Maximum number of frames to modify (-1 = unlimited)")
    ->group("General");

  app
    .add_option(
      "-j,--skip-frames", pointcloudmodifier::SKIP_FRAMES,
      "Number of frames to skip at the beginning")
    ->group("General");

  app.add_option("-s,--stride-frames", pointcloudmodifier::STRIDE_FRAMES, "Write every Nth frame")
    ->group("General");

  /* =========================
   * Transform options
   * ========================= */
  app
    .add_option(
      "--transform-file,--tf", pointcloudmodifier::TRANSFORM_FILE,
      "TXT file with additional transforms (frame_id r1 r2 r3 x r4 r5 r6 y r7 r8 r9 z)")
    ->group("Transforms");

  /* =========================
   * Filtering
   * ========================= */
  app
    .add_option(
      "--crop-box,--cb", pointcloudmodifier::CROPBOX, "Crop box [xmin ymin zmin xmax ymax zmax]")
    ->expected(6)
    ->type_name("FLOAT FLOAT FLOAT FLOAT FLOAT FLOAT")
    ->group("Filtering");

  app
    .add_option(
      "--crop-sphere,--cs", pointcloudmodifier::CROPSPHERE, "Crop to sphere with given radius")
    ->type_name("FLOAT")
    ->group("Filtering");

  app
    .add_option(
      "--crop-cylinder,--cc", pointcloudmodifier::CROPCYLINDER,
      "Crop to cylinder with given radius")
    ->type_name("FLOAT")
    ->group("Filtering");

  app.add_option("--voxel-filter,--vf", pointcloudmodifier::VOXELFILTER, "Voxel size [x y z]")
    ->expected(3)
    ->type_name("FLOAT FLOAT FLOAT")
    ->group("Filtering");

  app
    .add_option(
      "--outlier-radius-filter,--orf", pointcloudmodifier::OUTLIERRADIUSFILTER,
      "Radius outlier removal [radius min_neighbors]")
    ->expected(2)
    ->type_name("FLOAT INT")
    ->group("Filtering");

  app
    .add_option(
      "--outlier-stat-filter,--osf", pointcloudmodifier::OUTLIERSTATFILTER,
      "Statistical outlier removal [threshold mean_k]")
    ->expected(2)
    ->type_name("FLOAT INT")
    ->group("Filtering");

  app.footer(
    "\nExample:\n"
    "  ros2 run pointcloudmodifier modifier input/ out/ \n"
    "    --voxel-filter 0.1 0.1 0.1 -m 5\n");

  CLI11_PARSE(app, argc, argv);

  pointcloudmodifier::PointCloudModifier().run();

  rclcpp::shutdown();
  return 0;
}
