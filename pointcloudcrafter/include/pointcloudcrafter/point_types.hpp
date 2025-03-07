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
// https://pcl.readthedocs.io/projects/tutorials/en/latest/adding_custom_ptype.html#how-to-add-a-new-pointt-type
#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
namespace pointcloudcrafter
{
class pt
{
public:
  struct PointXYZIT
  {
    PCL_ADD_POINT4D;
    float intensity;
    std::uint32_t timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  } EIGEN_ALIGN16;
  struct PointXYZITd
  {
    PCL_ADD_POINT4D;
    float intensity;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  } EIGEN_ALIGN16;
};
}  // namespace pointcloudcrafter

POINT_CLOUD_REGISTER_POINT_STRUCT(
  pointcloudcrafter::pt::PointXYZIT,
  (float, x,
   x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint32_t, timestamp, time_us))

POINT_CLOUD_REGISTER_POINT_STRUCT(
  pointcloudcrafter::pt::PointXYZITd,
  (float, x,
   x)(float, y, y)(float, z, z)(float, intensity, intensity)(double, timestamp, timestamp))
