/*
 * Copyright 2024 2024 Markus Pielmeier, Florian Sauerbeck,
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

#pragma once

#include <math.h>

#include <algorithm>
#include <filesystem>  // NOLINT
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

// PCL
#define PCL_NO_PRECOMPILE
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using PointCloud = pcl::PCLPointCloud2;
namespace pointcloudmodifierlib
{
class Modifier
{
public:
  Modifier()
  : input_cloud(new pcl::PCLPointCloud2()), output_cloud(new pcl::PCLPointCloud2())
  {
  }
  ~Modifier() = default;

  // Loading/saving functions
  bool loadPCD(const std::string & file_path)
  {
    pcl::PCDReader reader;
    reader.read(file_path, *input_cloud);

    *output_cloud = *input_cloud;
    return true;
  }
  bool savePCD(const std::string & file_path)
  {
    std::filesystem::path p(file_path);
    std::filesystem::path dir = p.parent_path();
    std::string filename = p.filename().string();
    std::filesystem::path output_path = dir / filename;

    pcl::PCDWriter writer;
    if (
      writer.write(
        output_path.string(), output_cloud, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(),
        true) == -1) {
      std::cerr << "Failed to save PCD file: " << output_path.string() << std::endl;
      return false;
    }
    return true;
  }

  // Filter functions - all return reference to allow chaining
  Modifier & cropBox(const std::vector<double> & box_params, const bool & negative = false)
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
    boxFilter.setNegative(negative);
    boxFilter.filter(*output_cloud);
    return *this;
  }
  Modifier & cropSphere(const double & sphere_params, const bool & negative = false)
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
    extract.setNegative(negative);
    extract.filter(*output_cloud);

    return *this;
  }
  Modifier & cropCylinder(const double & zylinder_params, const bool & negative = false)
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
    extract.setNegative(negative);
    extract.filter(*output_cloud);

    return *this;
  }
  Modifier & voxelFilter(const std::vector<double> & voxel)
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
      applySubVoxelFilter(voxel, output_cloud, min_pt, max_pt, num_x, num_y, num_z);
    } else {
      applyVoxelFilter(voxel, output_cloud);
    }

    return *this;
  }
  Modifier & outlierRadiusFilter(const double & radius, const int & min_neighbors)
  {
    pcl::RadiusOutlierRemoval<PointCloud> radiusFilter;
    radiusFilter.setRadiusSearch(radius);
    radiusFilter.setMinNeighborsInRadius(min_neighbors);
    radiusFilter.setInputCloud(output_cloud);
    radiusFilter.filter(*output_cloud);
    return *this;
  }
  Modifier & outlierStatFilter(const double & threshold, const int & mean)
  {
    pcl::StatisticalOutlierRemoval<PointCloud> statFilter;
    statFilter.setMeanK(mean);
    statFilter.setStddevMulThresh(threshold);
    statFilter.setInputCloud(output_cloud);
    statFilter.filter(*output_cloud);
    return *this;
  }
  Modifier & timestampAnalyzer(const std::string & file_path)
  {
    std::vector<double> time_float{};
    std::vector<size_t> time_int{};
    for (size_t i = 0; i < output_cloud->data.size(); i += output_cloud->point_step) {
      for (size_t j = 0; j < output_cloud->fields.size(); ++j) {
        double tf{0.0};
        size_t ti{0};
        pcl::PCLPointField & field = output_cloud->fields[j];
        size_t point_offset = i + field.offset;
        if (field.name == "timestamp" || field.name == "time_stamp" || field.name == "t") {
          if (field.datatype == pcl::PCLPointField::FLOAT32) {
            float tmp_stamp;
            memcpy(&tmp_stamp, &output_cloud->data[point_offset], sizeof(float));
            tf = static_cast<double>(tmp_stamp);
          } else if (field.datatype == pcl::PCLPointField::FLOAT64) {
            memcpy(&tf, &output_cloud->data[point_offset], sizeof(double));
          } else if (field.datatype == pcl::PCLPointField::UINT32) {
            memcpy(&ti, &output_cloud->data[point_offset], sizeof(uint32_t));
          } else if (field.datatype == pcl::PCLPointField::UINT8 && field.count == 8) {
            ti = 0;
            for (size_t k = 0; k < 8; ++k) {
              ti |= static_cast<uint64_t>(output_cloud->data[point_offset + k]) << (8 * k);
            }
          } else if (field.datatype == pcl::PCLPointField::UINT8 && field.count == 4) {
            ti = 0;
            for (size_t k = 0; k < 4; ++k) {
              ti |= static_cast<uint32_t>(output_cloud->data[point_offset + k]) << (8 * k);
            }
          } else {
            std::cerr << "Error: Unknown datatype for timestamp field" << std::endl;
            return *this;
          }
        }
        if (tf > 0.0) {
          time_float.push_back(tf);
        } else if (ti > 0) {
          time_int.push_back(ti);
        }
      }
    }

    if (!time_float.empty()) {
      saveTimestamps<double>(time_float, file_path);

      // Basic statistics
      std::cout << "Float timestamps: " << time_float.size() << " entries" << std::endl;
      if (time_float.size() > 1) {
        double min_ts = *std::min_element(time_float.begin(), time_float.end());
        double max_ts = *std::max_element(time_float.begin(), time_float.end());
        std::cout << "  Min: " << min_ts << "\n  Max: " << max_ts
                  << "\n  Duration: " << (max_ts - min_ts) << std::endl;
      }
    }

    if (!time_int.empty()) {
      saveTimestamps<size_t>(time_int, file_path);

      // Basic statistics
      std::cout << "Integer timestamps: " << time_int.size() << " entries" << std::endl;
      if (time_int.size() > 1) {
        uint64_t min_ts = *std::min_element(time_int.begin(), time_int.end());
        uint64_t max_ts = *std::max_element(time_int.begin(), time_int.end());
        std::cout << "  Min: " << min_ts << "\n  Max: " << max_ts
                  << "\n  Duration: " << (max_ts - min_ts) << std::endl;
      }
    }

    return *this;
  }

  // Visualization
  void visualize()
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

  // Setters
  void setCloud(const PointCloud::Ptr & cloud)
  {
    *output_cloud = *cloud;
    *input_cloud = *cloud;
  }

  // Getters
  const PointCloud::Ptr getInputCloud() const { return input_cloud; }
  const PointCloud::Ptr getOutputCloud() const { return output_cloud; }

private:
  PointCloud::Ptr input_cloud;
  PointCloud::Ptr output_cloud;

  void applySubVoxelFilter(
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

  void applyVoxelFilter(const std::vector<double> & voxel, PointCloud::Ptr & cloud)
  {
    pcl::VoxelGrid<PointCloud> voxelFilter;
    voxelFilter.setLeafSize(voxel[0], voxel[1], voxel[2]);
    voxelFilter.setInputCloud(cloud);
    voxelFilter.filter(*cloud);
  }

  template <typename T>
  bool saveTimestamps(const std::vector<T> & timestamps, const std::string & output_path)
  {
    std::ofstream outfile;
    outfile.open(output_path);

    if (!outfile.is_open()) {
      std::cerr << "Failed to open file for writing: " << output_path << std::endl;
      return false;
    }

    for (const auto & ts : timestamps) {
      outfile << std::fixed << std::setprecision(9) << ts << std::endl;
    }

    outfile.close();
    std::cout << "Timestamps saved to: " << output_path << std::endl;
    return true;
  }
};
}  // namespace pointcloudmodifierlib
