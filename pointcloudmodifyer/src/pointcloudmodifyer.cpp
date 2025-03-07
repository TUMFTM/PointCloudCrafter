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
#include "pointcloudmodifyer.hpp"

#include <math.h>

#include <filesystem>
#include <iostream>
#include <limits>

// PCL
#include <pcl/cloud_iterator.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/visualization/pcl_visualizer.h>

using PointCloud = pcl::PCLPointCloud2;
namespace pointcloudmodifyer
{
Modifyer::Modifyer()
: input_cloud(new pcl::PCLPointCloud2()), output_cloud(new pcl::PCLPointCloud2())
{
}
Modifyer::~Modifyer() = default;
bool Modifyer::loadPCD(const std::string & file_path)
{
  pcl::PCDReader reader;
  reader.read(file_path, *input_cloud);

  *output_cloud = *input_cloud;
  std::cout << "Loading completed: " << input_cloud->width * input_cloud->height << " data points"
            << std::endl;
  return true;
}
bool Modifyer::savePCD(const std::string & file_path)
{
  std::string path = file_path;

  // If file_path is a directory, create a filename in that directory
  if (std::filesystem::is_directory(file_path)) {
    path = file_path + "/mod_cloud.pcd";
  }

  if (pcl::io::savePCDFile(path, *output_cloud) == -1) {
    std::cerr << "Failed to save PCD file: " << path << std::endl;
    return false;
  }

  std::cout << "PCD file saved to: " << path << std::endl;
  return true;
}
void Modifyer::visualize()
{
  // Convert to PointXYZ for visualization
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_xyz(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_xyz(new pcl::PointCloud<pcl::PointXYZ>());

  pcl::fromPCLPointCloud2(*input_cloud, *input_xyz);
  pcl::fromPCLPointCloud2(*output_cloud, *output_xyz);

  pcl::visualization::PCLVisualizer viewer("PCD Viewer");

  // Add input cloud with white color
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> input_color(
    input_xyz, 255, 255, 255);
  viewer.addPointCloud(input_xyz, input_color, "input_cloud");
  viewer.setPointCloudRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "input_cloud");

  // Add output cloud with red color
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> output_color(
    output_xyz, 230, 20, 20);
  viewer.addPointCloud(output_xyz, output_color, "output_cloud");
  viewer.setPointCloudRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "output_cloud");

  // Additional visualization settings
  viewer.addCoordinateSystem(1.0, "cloud", 0);
  viewer.setBackgroundColor(0.15, 0.15, 0.15, 0);

  std::cout << "Input_cloud: white | Output_cloud: red" << std::endl;

  // Spin until window is closed
  viewer.spin();
}
Modifyer & Modifyer::cropBox(const std::vector<double> & box_params)
{
  if (box_params.size() < 6) {
    std::cerr << "Error: cropBox requires 6 parameters (min_x, min_y, min_z, max_x, max_y, max_z)"
              << std::endl;
    return *this;
  }

  pcl::CropBox<PointCloud> boxFilter;
  boxFilter.setMin(Eigen::Vector4f(box_params[0], box_params[1], box_params[2], 1.0));
  boxFilter.setMax(Eigen::Vector4f(box_params[3], box_params[4], box_params[5], 1.0));
  boxFilter.setInputCloud(output_cloud);
  boxFilter.filter(*output_cloud);
  return *this;
}
Modifyer & Modifyer::cropSphere(const double & sphere_params)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(*output_cloud, *tmp);

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

  for (size_t i = 0; i < tmp->size(); ++i) {
    if (tmp->points[i].getVector3fMap().norm() < sphere_params) {
      inliers->indices.push_back(i);
    }
  }

  pcl::ExtractIndices<pcl::PCLPointCloud2> extract;
  extract.setInputCloud(output_cloud);
  extract.setIndices(inliers);
  extract.filter(*output_cloud);

  return *this;
}
Modifyer & Modifyer::cropZylinder(const double & zylinder_params)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(*output_cloud, *tmp);

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

  for (size_t i = 0; i < tmp->size(); ++i) {
    auto point = tmp->points[i].getVector3fMap();
    double r = std::sqrt(point.x() * point.x() + point.y() * point.y());
    if (r < zylinder_params) {
      inliers->indices.push_back(i);
    }
  }

  pcl::ExtractIndices<pcl::PCLPointCloud2> extract;
  extract.setInputCloud(output_cloud);
  extract.setIndices(inliers);
  extract.filter(*output_cloud);

  return *this;
}
Modifyer & Modifyer::voxelfilter(const std::vector<double> & voxel)
{
  if (voxel.size() < 3) {
    std::cerr << "Error: Voxel filter requires 3 parameters (voxel_x, voxel_y, voxel_z)"
              << std::endl;
    return *this;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(*output_cloud, *tmp);

  // Compute min_pt and max_pt
  pcl::PointXYZ min_pt, max_pt;
  pcl::getMinMax3D(*tmp, min_pt, max_pt);

  // Figure out how many cells we’d have in each dimension
  const auto num_x = static_cast<uint64_t>(std::floor((max_pt.x - min_pt.x) / voxel[0]));
  const auto num_y = static_cast<uint64_t>(std::floor((max_pt.y - min_pt.y) / voxel[1]));
  const auto num_z = static_cast<uint64_t>(std::floor((max_pt.z - min_pt.z) / voxel[2]));
  const auto total = num_x * num_y * num_z;

  // If the total # of voxels would exceed int32_t, apply subvoxel
  if (total > static_cast<uint64_t>(INT32_MAX)) {
    applySubVoxelfilter(voxel, output_cloud, min_pt, max_pt, num_x, num_y, num_z);
  } else {
    applyVoxelfilter(voxel, output_cloud);
  }

  return *this;
}
void Modifyer::applyVoxelfilter(const std::vector<double> & voxel, PointCloud::Ptr & cloud)
{
  pcl::VoxelGrid<PointCloud> voxelFilter;
  voxelFilter.setLeafSize(voxel[0], voxel[1], voxel[2]);
  voxelFilter.setInputCloud(cloud);
  voxelFilter.filter(*cloud);
}
void Modifyer::applySubVoxelfilter(
  const std::vector<double> & voxel, PointCloud::Ptr & cloud, const pcl::PointXYZ & min_pt,
  const pcl::PointXYZ & max_pt, const uint64_t num_x, const uint64_t num_y, const uint64_t num_z)
{
  PointCloud::Ptr tpc = std::make_shared<PointCloud>();
  PointCloud::Ptr apc = std::make_shared<PointCloud>();
  uint64_t num = std::floor(std::cbrt((num_x * num_y * num_z) / INT32_MAX)) + 1;
  uint64_t iter = 0;

  for (uint64_t ix = 0; ix < num; ix++) {
    for (uint64_t iy = 0; iy < num; iy++) {
      for (uint64_t iz = 0; iz < num; iz++) {
        std::cout << "\rSubvoxel split: " << ++iter << " of "
                  << static_cast<uint64_t>(std::pow(num, 3)) << std::flush;
        pcl::CropBox<PointCloud> box;
        pcl::PointXYZ tmin = {
          min_pt.x + (ix * (max_pt.x - min_pt.x) / num),
          min_pt.y + (iy * (max_pt.y - min_pt.y) / num),
          min_pt.z + (iz * (max_pt.z - min_pt.z) / num)};
        pcl::PointXYZ tmax = {
          tmin.x + (max_pt.x - min_pt.x) / num, tmin.y + (max_pt.y - min_pt.y) / num,
          tmin.z + (max_pt.z - min_pt.z) / num};

        box.setMin(Eigen::Vector4f(tmin.x, tmin.y, tmin.z, 1.0));
        box.setMax(Eigen::Vector4f(tmax.x, tmax.y, tmax.z, 1.0));
        box.setInputCloud(cloud);
        box.filter(*tpc);

        pcl::VoxelGrid<PointCloud> grid;
        grid.setInputCloud(tpc);
        grid.setLeafSize(voxel[0], voxel[1], voxel[2]);
        grid.filter(*tpc);

        *apc += *tpc;
      }
    }
  }
  std::cout << std::endl;
  *output_cloud = *apc;
}

}  // namespace pointcloudmodifyer
