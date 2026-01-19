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

#include "cli/CLI11.hpp"
#include "cli/cli_config.hpp"
#include "pointcloudcrafter_rosbag/pointcloudcrafter_rosbag.hpp"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  CLI::App app{"pointcloudcrafter_rosbag"};
  app.name("ros2 run pointcloudcrafter rosbag");

  config::RosbagConfig cfg;
  cfg.add_cli_options(&app);

  app.footer(
    "\nExample:\n"
    "  ros2 run pointcloudcrafter rosbag bag.mcap out/ /points_raw \n"
    "    --voxel-filter 0.1 0.1 0.1 --stride-frames 5\n");

  CLI11_PARSE(app, argc, argv);

  pointcloudcrafter::Rosbag(cfg).run();

  rclcpp::shutdown();
  return 0;
}
