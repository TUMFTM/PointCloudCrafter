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
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "formats.hpp"

using PointCloud = pcl::PCLPointCloud2;
namespace pointcloudmodifierlib
{
/**
 * @brief Modifier class for point cloud processing
 */
class Modifier
{
public:
  Modifier()
  : input_cloud(new pcl::PCLPointCloud2()), output_cloud(new pcl::PCLPointCloud2())
  {
  }
  ~Modifier() = default;

  // Loading point clouds
  /**
   * @brief Load a point cloud file based on the specified format
   * @param file_path Path to the file
   * @param format The file format
   * @return True if successful, false otherwise
   */
  bool load(const std::string & file_path,
    pointcloudcrafter::tools::formats::FileFormat format)
  {
    switch (format) {
      case pointcloudcrafter::tools::formats::FileFormat::PCD:
        return load_pcd(file_path);
      case pointcloudcrafter::tools::formats::FileFormat::PLY:
        return load_ply(file_path);
      case pointcloudcrafter::tools::formats::FileFormat::XYZ:
        return load_xyz(file_path);
      case pointcloudcrafter::tools::formats::FileFormat::KITTI:
        return load_kitti(file_path);
      case pointcloudcrafter::tools::formats::FileFormat::NUSCENES:
        return load_nuscenes(file_path);
      case pointcloudcrafter::tools::formats::FileFormat::OBJ:
        return load_obj(file_path);
      default:
        std::cerr << "Error: Unknown file format" << std::endl;
        return false;
    }
  }
  /**
   * @brief Save point cloud to file based on the specified format
   * @param file_path Path to save the file
   * @param format The file format
   * @return True if successful, false otherwise
   */
  bool save(const std::string & file_path,
    pointcloudcrafter::tools::formats::FileFormat format)
  {
    switch (format) {
      case pointcloudcrafter::tools::formats::FileFormat::PCD:
        return save_pcd(file_path);
      case pointcloudcrafter::tools::formats::FileFormat::PLY:
        return save_ply(file_path);
      case pointcloudcrafter::tools::formats::FileFormat::XYZ:
        return save_xyz(file_path);
      case pointcloudcrafter::tools::formats::FileFormat::KITTI:
        return save_kitti(file_path);
      case pointcloudcrafter::tools::formats::FileFormat::NUSCENES:
        return save_nuscenes(file_path);
      default:
        std::cerr << "Error: Unknown file format" << std::endl;
        return false;
    }
  }

  // Filtering functions
  /**
   * @brief Crop box filter
   * @param box_params Vector of 6 doubles defining the min and max points of the box
   * @param negative If true, removes points inside the box
   * @return Reference to the Modifier object
   */
  Modifier & cropBox(const std::vector<double> & box_params, const bool & negative = false)
  {
    pcl::CropBox<PointCloud> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(box_params[0], box_params[1], box_params[2], 1.0));
    boxFilter.setMax(Eigen::Vector4f(box_params[3], box_params[4], box_params[5], 1.0));
    boxFilter.setInputCloud(output_cloud);
    boxFilter.setNegative(negative);
    boxFilter.filter(*output_cloud);
    return *this;
  }
  /**
   * @brief Crop sphere filter
   * @param sphere_params Radius of the sphere
   * @param negative If true, removes points inside the sphere
   * @return Reference to the Modifier object
   */
  Modifier & cropSphere(const double & sphere_params, const bool & negative = false)
  {
    const auto radius_sq = sphere_params * sphere_params;
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*output_cloud, *tmp);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    inliers->indices.reserve(tmp->size());

    for (size_t i = 0; i < tmp->size(); ++i) {
      if (tmp->points[i].getVector3fMap().squaredNorm() < radius_sq) {
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
  /**
   * @brief Crop cylinder filter
   * @param zylinder_params Radius of the cylinder
   * @param negative If true, removes points inside the cylinder
   * @return Reference to the Modifier object
   */
  Modifier & cropCylinder(const double & zylinder_params, const bool & negative = false)
  {
    const auto radius_sq = zylinder_params * zylinder_params;
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*output_cloud, *tmp);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    inliers->indices.reserve(tmp->size());

    for (size_t i = 0; i < tmp->size(); ++i) {
      auto point = tmp->points[i].getVector3fMap();
      auto r = point.x() * point.x() + point.y() * point.y();
      if (r < radius_sq) {
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
  /**
   * @brief Voxel grid filter with subvoxel handling
   * @param voxel Vector of 3 doubles defining the voxel size in x, y, z
   * @return Reference to the Modifier object
   */
  Modifier & voxelFilter(const std::vector<double> & voxel)
  {
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
  /**
   * @brief Outlier radius filter
   * @param radius Radius for the filter
   * @param min_neighbors Minimum number of neighbors to be considered an inlier
   * @return Reference to the Modifier object
   */
  Modifier & outlierRadiusFilter(const double & radius, const int & min_neighbors)
  {
    pcl::RadiusOutlierRemoval<PointCloud> radiusFilter;
    radiusFilter.setRadiusSearch(radius);
    radiusFilter.setMinNeighborsInRadius(min_neighbors);
    radiusFilter.setInputCloud(output_cloud);
    radiusFilter.filter(*output_cloud);
    return *this;
  }
  /**
   * @brief Outlier statistical filter
   * @param threshold Standard deviation multiplier threshold
   * @param mean Number of nearest neighbors to use for mean distance estimation
   * @return Reference to the Modifier object
   */
  Modifier & outlierStatFilter(const double & threshold, const int & mean)
  {
    pcl::StatisticalOutlierRemoval<PointCloud> statFilter;
    statFilter.setMeanK(mean);
    statFilter.setStddevMulThresh(threshold);
    statFilter.setInputCloud(output_cloud);
    statFilter.filter(*output_cloud);
    return *this;
  }

  // Transforms
  /**
   * @brief Apply transformation to the point cloud
   * @param transformation Eigen Affine3d transformation matrix
   * @return Reference to the Modifier object
   */
  Modifier & transform(const Eigen::Affine3d & transformation)
  {
    // Find x, y, z field offsets
    int x_offset = -1, y_offset = -1, z_offset = -1;
    for (const auto & field : output_cloud->fields) {
      if (field.name == "x") x_offset = field.offset;
      else if (field.name == "y") y_offset = field.offset;
      else if (field.name == "z") z_offset = field.offset;
    }

    Eigen::Affine3f transform_f = transformation.cast<float>();

    // Transform each point in-place
    for (size_t i = 0; i < output_cloud->data.size(); i += output_cloud->point_step) {
      float x, y, z;
      memcpy(&x, &output_cloud->data[i + x_offset], sizeof(float));
      memcpy(&y, &output_cloud->data[i + y_offset], sizeof(float));
      memcpy(&z, &output_cloud->data[i + z_offset], sizeof(float));

      Eigen::Vector3f pt(x, y, z);
      Eigen::Vector3f pt_transformed = transform_f * pt;

      memcpy(&output_cloud->data[i + x_offset], &pt_transformed.x(), sizeof(float));
      memcpy(&output_cloud->data[i + y_offset], &pt_transformed.y(), sizeof(float));
      memcpy(&output_cloud->data[i + z_offset], &pt_transformed.z(), sizeof(float));
    }

    return *this;
  }

  // Analysis
  /**
   * @brief Save timestamps to a file and print basic statistics
   * @param file_path Path to save the timestamps
   * @return Reference to the Modifier object
   */
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
  /**
   * @brief Visualize input and output point clouds
   */
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
  /**
   * @brief Set point cloud directly
   * @param cloud Point cloud to set
   */
  void setCloud(const PointCloud::Ptr & cloud)
  {
    *output_cloud = *cloud;
    *input_cloud = *cloud;
  }

  // Getters
  /**
   * @brief Get input point cloud
   * @return Pointer to the input point cloud
   */
  const PointCloud::Ptr getInputCloud() const { return input_cloud; }
  /**
   * @brief Get output point cloud
   * @return Pointer to the output point cloud
   */
  const PointCloud::Ptr getOutputCloud() const { return output_cloud; }

private:
  PointCloud::Ptr input_cloud;
  PointCloud::Ptr output_cloud;

  /**
   * @brief Load PCD file
   * @param file_path Path to the PCD file
   * @return True if successful, false otherwise
   */
  bool load_pcd(const std::string & file_path)
  {
    pcl::PCDReader reader;
    if (reader.read(file_path, *input_cloud) == -1) {
      std::cerr << "Error: Failed to load PCD file: " << file_path << std::endl;
      return false;
    }
    *output_cloud = *input_cloud;
    return true;
  }
  /**
   * @brief Load a PLY file into PCLPointCloud2
   * @param file_path Path to the PLY file
   * @return True if successful, false otherwise
   */
  bool load_ply(const std::string & file_path)
  {
    pcl::PLYReader reader;
    if (reader.read(file_path, *input_cloud) == -1) {
      std::cerr << "Error: Failed to load PLY file: " << file_path << std::endl;
      return false;
    }
    *output_cloud = *input_cloud;
    return true;
  }
  /**
   * @brief Load an XYZ ASCII file into PCLPointCloud2
   * @param file_path Path to the XYZ file
   * @return True if successful, false otherwise
   */
  bool load_xyz(const std::string & file_path)
  {
    std::ifstream file(file_path);
    if (!file.is_open()) {
      std::cerr << "Error: Failed to open XYZ file: " << file_path << std::endl;
      return false;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
    std::string line;

    while (std::getline(file, line)) {
      if (line.empty() || line[0] == '#') continue;

      std::istringstream iss(line);
      float x, y, z;
      if (iss >> x >> y >> z) {
        tmp->push_back(pcl::PointXYZ(x, y, z));
      }
    }

    tmp->width = tmp->size();
    tmp->height = 1;
    tmp->is_dense = true;

    pcl::toPCLPointCloud2(*tmp, *input_cloud);
    *output_cloud = *input_cloud;
    return true;
  }
  /**
   * @brief Load a KITTI binary file into PCLPointCloud2
   * 
   * KITTI format: binary file with points as (x, y, z, intensity) float32
   * 
   * @param file_path Path to the KITTI .bin file
   * @return True if successful, false otherwise
   */
  bool load_kitti(const std::string & file_path)
  {
    std::ifstream file(file_path, std::ios::binary);
    if (!file.is_open()) {
      std::cerr << "Error: Failed to open KITTI file: " << file_path << std::endl;
      return false;
    }

    // Get file size
    file.seekg(0, std::ios::end);
    const size_t file_size = file.tellg();
    file.seekg(0, std::ios::beg);

    // KITTI format: 4 floats per point (x, y, z, intensity)
    constexpr size_t point_size = 4 * sizeof(float);
    const size_t num_points = file_size / point_size;

    // Read all data
    std::vector<float> buffer(num_points * 4);
    file.read(reinterpret_cast<char*>(buffer.data()), file_size);

    // Convert to PointXYZI
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZI>);
    tmp->reserve(num_points);

    for (size_t i = 0; i < num_points; ++i) {
      pcl::PointXYZI point;
      point.x = buffer[i * 4 + 0];
      point.y = buffer[i * 4 + 1];
      point.z = buffer[i * 4 + 2];
      point.intensity = buffer[i * 4 + 3];
      tmp->push_back(point);
    }

    tmp->width = tmp->size();
    tmp->height = 1;
    tmp->is_dense = true;

    pcl::toPCLPointCloud2(*tmp, *input_cloud);
    *output_cloud = *input_cloud;
    return true;
  }
  /**
   * @brief Load a nuScenes binary file into PCLPointCloud2
   * 
   * nuScenes format: binary file with points as (x, y, z, intensity, ring) float32
   * 
   * @param file_path Path to the nuScenes .bin file
   * @return True if successful, false otherwise
   */
  bool load_nuscenes(const std::string & file_path)
  {
    std::ifstream file(file_path, std::ios::binary);
    if (!file.is_open()) {
      std::cerr << "Error: Failed to open nuScenes file: " << file_path << std::endl;
      return false;
    }

    // Get file size
    file.seekg(0, std::ios::end);
    const size_t file_size = file.tellg();
    file.seekg(0, std::ios::beg);

    // nuScenes format: 5 floats per point (x, y, z, intensity, ring)
    constexpr size_t point_size = 5 * sizeof(float);
    const size_t num_points = file_size / point_size;

    // Read all data
    std::vector<float> buffer(num_points * 5);
    file.read(reinterpret_cast<char*>(buffer.data()), file_size);

    // Create custom point cloud with ring field
    // Using PointXYZI and storing ring in a separate structure
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZI>);
    tmp->reserve(num_points);

    for (size_t i = 0; i < num_points; ++i) {
      pcl::PointXYZI point;
      point.x = buffer[i * 5 + 0];
      point.y = buffer[i * 5 + 1];
      point.z = buffer[i * 5 + 2];
      point.intensity = buffer[i * 5 + 3];
      // Note: ring (buffer[i * 5 + 4]) is lost in PointXYZI
      // For full support, use a custom point type
      tmp->push_back(point);
    }

    tmp->width = tmp->size();
    tmp->height = 1;
    tmp->is_dense = true;

    pcl::toPCLPointCloud2(*tmp, *input_cloud);
    *output_cloud = *input_cloud;
    return true;
  }
  /**
   * @brief Load a OBJ file into PCLPointCloud2
   * @param file_path Path to the OBJ file
   * @return True if successful, false otherwise
   */
  bool load_obj(const std::string & file_path)
  {
    pcl::OBJReader reader;
    if (reader.read(file_path, *input_cloud) == -1) {
      std::cerr << "Error: Failed to load OBJ file: " << file_path << std::endl;
      return false;
    }
    *output_cloud = *input_cloud;
    return true;
  }

  /**
   * @brief Save point cloud to PCD file
   * @param file_path Path to save the PCD file
   * @return True if successful, false otherwise
   */
  bool save_pcd(const std::string & file_path)
  {
    pcl::PCDWriter writer;
    if (writer.write(file_path, *output_cloud, Eigen::Vector4f::Zero(),
          Eigen::Quaternionf::Identity(), true) == -1) {
      std::cerr << "Error: Failed to save PCD file: " << file_path << std::endl;
      return false;
    }
    return true;
  }
  /**
   * @brief Save point cloud to PLY file
   * @param file_path Path to save the PLY file
   * @return True if successful, false otherwise
   */
  bool save_ply(const std::string & file_path)
  {
    pcl::PLYWriter writer;
    if (writer.write(file_path, *output_cloud, Eigen::Vector4f::Zero(),
          Eigen::Quaternionf::Identity(), true, true) == -1) {
      std::cerr << "Error: Failed to save PLY file: " << file_path << std::endl;
      return false;
    }
    return true;
  }
  /**
   * @brief Save point cloud to XYZ ASCII file
   * @param file_path Path to save the XYZ file
   * @return True if successful, false otherwise
   */
  bool save_xyz(const std::string & file_path)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*output_cloud, *tmp);

    std::ofstream file(file_path);
    if (!file.is_open()) {
      std::cerr << "Error: Failed to open XYZ file for writing: " << file_path << std::endl;
      return false;
    }

    file << std::fixed << std::setprecision(6);
    for (const auto & point : tmp->points) {
      file << point.x << " " << point.y << " " << point.z << "\n";
    }
    return true;
  }
  /**
   * @brief Save point cloud to KITTI binary file
   * @param file_path Path to save the KITTI .bin file
   * @return True if successful, false otherwise
   */
  bool save_kitti(const std::string & file_path)
  {
    std::ofstream file(file_path, std::ios::binary);
    if (!file.is_open()) {
      std::cerr << "Error: Failed to open KITTI file for writing: " << file_path << std::endl;
      return false;
    }

    // Check if intensity field exists
    bool has_intensity = std::any_of(
      output_cloud->fields.begin(), output_cloud->fields.end(),
      [](const pcl::PCLPointField & f) { return f.name == "intensity"; });

    if (has_intensity) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::fromPCLPointCloud2(*output_cloud, *tmp);
      for (const auto & point : tmp->points) {
        float data[4] = {point.x, point.y, point.z, point.intensity};
        file.write(reinterpret_cast<char *>(data), sizeof(data));
      }
    } else {
      pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromPCLPointCloud2(*output_cloud, *tmp);
      for (const auto & point : tmp->points) {
        float data[4] = {point.x, point.y, point.z, 0.0f};
        file.write(reinterpret_cast<char *>(data), sizeof(data));
      }
    }
    return true;
  }
  /**
   * @brief Save point cloud to nuScenes binary file
   * @param file_path Path to save the nuScenes .bin file
   * @return True if successful, false otherwise
   */
  bool save_nuscenes(const std::string & file_path)
  {
    std::ofstream file(file_path, std::ios::binary);
    if (!file.is_open()) {
      std::cerr << "Error: Failed to open nuScenes file for writing: " << file_path << std::endl;
      return false;
    }

    bool has_intensity = std::any_of(
      output_cloud->fields.begin(), output_cloud->fields.end(),
      [](const pcl::PCLPointField & f) { return f.name == "intensity"; });

    if (has_intensity) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::fromPCLPointCloud2(*output_cloud, *tmp);
      for (const auto & point : tmp->points) {
        float data[5] = {point.x, point.y, point.z, point.intensity, 0.0f};
        file.write(reinterpret_cast<char *>(data), sizeof(data));
      }
    } else {
      pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromPCLPointCloud2(*output_cloud, *tmp);
      for (const auto & point : tmp->points) {
        float data[5] = {point.x, point.y, point.z, 0.0f, 0.0f};
        file.write(reinterpret_cast<char *>(data), sizeof(data));
      }
    }
    return true;
  }

  /**
   * @brief Apply subvoxel filter to the point cloud
   * @param voxel Vector of 3 doubles defining the voxel size in x, y, z
   * @param cloud Pointer to the point cloud
   * @param min_pt Minimum point of the bounding box
   * @param max_pt Maximum point of the bounding box
   * @param num_x Number of voxels in x direction
   * @param num_y Number of voxels in y direction
   * @param num_z Number of voxels in z direction
   */
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

  /**
   * @brief Apply voxel filter to the point cloud
   * @param voxel Vector of 3 doubles defining the voxel size in x, y, z
   * @param cloud Pointer to the point cloud
   */
  void applyVoxelFilter(const std::vector<double> & voxel, PointCloud::Ptr & cloud)
  {
    pcl::VoxelGrid<PointCloud> voxelFilter;
    voxelFilter.setLeafSize(voxel[0], voxel[1], voxel[2]);
    voxelFilter.setInputCloud(cloud);
    voxelFilter.filter(*cloud);
  }

  /**
   * @brief Save timestamps to a file
   * @param timestamps Vector of timestamps
   * @param output_path Path to save the timestamps
   * @return True if successful, false otherwise
   */
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
