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
#include "cli_config.hpp"
#include "pcdmodifier.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  CLI::App app{"pointcloudmodifier"};
  app.name("ros2 run pointcloudmodifier modifier");

  config::PcdModifierConfig cfg;
  cfg.add_cli_options(&app);

  app.footer(
    "\nExample:\n"
    "  ros2 run pointcloudmodifier modifier input/ out/ \n"
    "    --voxel-filter 0.1 0.1 0.1 -m 5\n");

  CLI11_PARSE(app, argc, argv);

  pointcloudmodifier::PointCloudModifier(cfg).run();

  rclcpp::shutdown();
  return 0;
}
