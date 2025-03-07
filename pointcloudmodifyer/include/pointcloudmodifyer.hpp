// Copyright 2024 Maximilian Leitenstern, Dominik Kulmer

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#pragma once

#include <string>
#include <tuple>
#include <vector>
#include <memory>

// PCL
#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>

using PointCloud = pcl::PCLPointCloud2;

namespace pointcloudmodifyer {

class Modifyer {
public:
    Modifyer();
    ~Modifyer();

    // Loading/saving functions
    bool loadPCD(const std::string& file_path);
    bool savePCD(const std::string& file_path);

    // Filter functions - all return reference to allow chaining
    Modifyer& cropBox(const std::vector<double>& box_params);
    Modifyer& cropSphere(const double& sphere_params);
    Modifyer& cropZylinder(const double& zylinder_params);
    Modifyer& voxelfilter(const std::vector<double>& voxel);


    // Visualization
    void visualize();

    // Setters
    void setCloud(const PointCloud::Ptr& cloud) { *output_cloud = *cloud; *input_cloud = *cloud; }

    // Getters
    const PointCloud::Ptr getInputCloud() const { return input_cloud; }
    const PointCloud::Ptr getOutputCloud() const { return output_cloud; }

private:
    PointCloud::Ptr input_cloud;
    PointCloud::Ptr output_cloud;

    void applySubVoxelfilter(const std::vector<double>& voxel, PointCloud::Ptr& cloud,
      const pcl::PointXYZ& min_pt, const pcl::PointXYZ& max_pt, const uint64_t num_x,
      const uint64_t num_y, const uint64_t num_z);

    void applyVoxelfilter(const std::vector<double>& voxel, PointCloud::Ptr& cloud);
};

}  // namespace pointcloudmodifyer
