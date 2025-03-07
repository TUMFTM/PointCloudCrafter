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
#include "pointcloudcrafter/rosbag_to_gnss.hpp"
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    CLI::App app{"rosbag_to_gnss"};
    app.option_defaults()->always_capture_default();

    app.add_option("bag-path", tum::mapping::bag_path, "Path to ROS2 Bag")->required();
    app.add_option("out-dir", tum::mapping::out_dir, "Output directory for gnss data")->required();

    app.add_option("topic-names", tum::mapping::topic_name, "Name of ROS2 GNSS topic")->required();
    app.add_option("topic-type", tum::mapping::msg_type, "Type of ROS2 GNSS msg")->required();

    app.add_flag("--dump, -d", tum::mapping::dump_gnss, "Dump all GNSS msgs to one file");
    app.add_option("--origin, -o", tum::mapping::origin_wgs84,
                   "Calculate the relative position in local cartesian to an origin [lat lon hgt]");
    app.add_flag("--utm, -u", tum::mapping::convert_utm, "Convert GNSS msgs to UTM format");
    app.add_option("--antenna, -a", tum::mapping::gnss_antenna,
                   "Transform coordinates to base_link from specified GNSS antenna - requires /tf "
                   "and /tf_static");

    CLI11_PARSE(app, argc, argv);

    tum::mapping::RosbagToGnss().run();

    rclcpp::shutdown();
    return 0;
}
