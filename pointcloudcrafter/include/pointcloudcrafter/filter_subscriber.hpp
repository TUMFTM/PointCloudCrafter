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

#pragma once

#include <memory>
#include <string>

#include "message_filters/message_event.h"
#include "message_filters/simple_filter.h"
#include "rosbag_reader.hpp"
namespace pointcloudcrafter::tools {
template <typename T>
class BagSubscriber : public message_filters::SimpleFilter<T> {
public:
    BagSubscriber(const std::string &topic_name, RosbagReader &reader, bool bag_time = false) {
        reader.add_listener<T>(topic_name, [this, bag_time](const RosbagReaderMsg<T> &msg) {
            auto msg_ptr = std::make_shared<T>(msg.ros_msg);

            if (bag_time) {
                msg_ptr->header.stamp.sec = static_cast<int>(msg.bag_msg.time_stamp / 1000000000L);
                msg_ptr->header.stamp.nanosec = msg.bag_msg.time_stamp % 1000000000L;
            }

            this->signalMessage(msg_ptr);
        });
    }
};
}  // namespace pointcloudcrafter::tools
