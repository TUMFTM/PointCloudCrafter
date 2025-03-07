/*
 * Copyright (C) 2024 Markus Pielmeier, Florian Sauerbeck, Dominik Kulmer, Maximilian Leitenstern
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <rclcpp/rclcpp.hpp>

#include "CLI11.hpp"
#include "pointcloudcrafter/pointcloudcrafter.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  CLI::App app{"rosbag_to_pcd"};
  app.option_defaults()->always_capture_default();

  app.add_option("bag-path", pointcloudcrafter::bag_path, "Path to ROS2 Bag")->required();
  app.add_option("out-dir", pointcloudcrafter::out_dir, "Output directory for .pcd files")
    ->required();
  app
    .add_option(
      "topic-names", pointcloudcrafter::topic_names, "Name of ROS topic of type PointCloud2")
    ->required();

  app.add_option(
    "--target-frame, -t", pointcloudcrafter::target_frame,
    "Target TF2 frame which all pointclouds are transformed to.");
  app.add_option(
    "--max-frames, -m", pointcloudcrafter::max_frames,
    "Maximum number of frames to extract. Use -1 for no limit");
  app.add_option(
    "--skip-frames, -j", pointcloudcrafter::skip_frames,
    "Number of frames to skip at the beginning");
  app.add_option(
    "--stride-frames, -s", pointcloudcrafter::stride_frames, "Write every Nth frame to file");
  app.add_option(
    "--sensor-number-field, -f", pointcloudcrafter::sensor_number_field,
    "Field name to write the sensor number to. Leave empty to skip");
  app.add_option(
    "--transform-file, --tf", pointcloudcrafter::transform_file,
    "Path to yaml file with extra transforms");
  app.add_option(
    "--geometric-filtering, --gf", pointcloudcrafter::geometric_filtering,
    "Remove rectangular box around orgin [x_min y_min z_min x_max y_max z_max]");
  app.add_flag(
    "--sequential-name", pointcloudcrafter::sequential_names,
    "Name files sequentially instead of based on their timestamp");
  app.add_flag(
    "--bag-time, -b", pointcloudcrafter::bag_time, "Use bag timestamps instead of header");
  app.add_flag(
    "--pie-filter, --pf", pointcloudcrafter::pie_filter,
    "Do circle segment filtering to cut out chase vehicle");

  CLI11_PARSE(app, argc, argv);

  pointcloudcrafter::PointCloudCrafter().run();

  rclcpp::shutdown();
  return 0;
}
