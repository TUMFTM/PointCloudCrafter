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

#include "pointcloudcrafter/utils.hpp"

#include <fmt/format.h>

#include <cstdint>
#include <fstream>
namespace pointcloudcrafter::tools::utils {
constexpr auto one_billion = 1000000000L;
void save_timestamps(const std::string &filename, const std::vector<uint64_t> &timestamps) {
    std::ofstream gps_file;
    gps_file.open(filename);
    for (uint64_t timestamp : timestamps) {
        gps_file << fmt::format("{} {:09d}\n", timestamp / one_billion, timestamp % one_billion);
    }
}
uint64_t timestamp_from_ros(const builtin_interfaces::msg::Time &ros) {
    return static_cast<uint64_t>(ros.sec) * one_billion + static_cast<uint64_t>(ros.nanosec);
}
builtin_interfaces::msg::Time timestamp_to_ros(uint64_t stamp) {
    builtin_interfaces::msg::Time ros;
    ros.sec = static_cast<int32_t>(stamp / one_billion);
    ros.nanosec = static_cast<int32_t>(stamp % one_billion);
    return ros;
}
}  // namespace pointcloudcrafter::tools::utils
