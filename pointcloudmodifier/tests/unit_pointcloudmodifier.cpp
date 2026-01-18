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

#include <gtest/gtest.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "pointcloudmodifier.hpp"
/**
 * @brief Helper function to create a test point cloud with known points
 * Creates a grid of points from -5 to 5 in each dimension (11x11x11 = 1331 points)
 */
pcl::PCLPointCloud2::Ptr createTestPointCloud()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  // Create a grid of points from -5 to 5 in each dimension
  for (int x = -5; x <= 5; ++x) {
    for (int y = -5; y <= 5; ++y) {
      for (int z = -5; z <= 5; ++z) {
        cloud->points.push_back(
          pcl::PointXYZ(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)));
      }
    }
  }
  cloud->width = cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = true;
  pcl::PCLPointCloud2::Ptr cloud2(new pcl::PCLPointCloud2);
  pcl::toPCLPointCloud2(*cloud, *cloud2);
  return cloud2;
}
/**
 * @brief Helper function to get the number of points in a PCLPointCloud2
 */
size_t getPointCount(const pcl::PCLPointCloud2::Ptr & cloud)
{
  if (!cloud || cloud->width == 0) {
    return 0;
  }
  return cloud->width * cloud->height;
}
/**
 * @brief Test setCloud functionality
 */
TEST(pointcloudmodifier, setCloud)
{
  pointcloudmodifierlib::Modifier modifier;
  auto test_cloud = createTestPointCloud();

  modifier.setCloud(test_cloud);

  auto input_cloud = modifier.getInputCloud();
  auto output_cloud = modifier.getOutputCloud();

  EXPECT_EQ(getPointCount(input_cloud), 1331) << "Input cloud should have 1331 points";
  EXPECT_EQ(getPointCount(output_cloud), 1331) << "Output cloud should have 1331 points";
  EXPECT_EQ(getPointCount(input_cloud), getPointCount(output_cloud))
    << "Input and output should have same point count initially";
}
/**
 * @brief Test cropBox filter
 */
TEST(pointcloudmodifier, cropBox)
{
  pointcloudmodifierlib::Modifier modifier;
  auto test_cloud = createTestPointCloud();
  modifier.setCloud(test_cloud);

  size_t initial_points = getPointCount(modifier.getOutputCloud());
  EXPECT_EQ(initial_points, 1331) << "Initial point count should be 1331";

  // Crop to box from -2 to 2 in each dimension (5x5x5 = 125 points)
  std::vector<double> box_params = {-2.0, -2.0, -2.0, 2.0, 2.0, 2.0};
  modifier.cropBox(box_params);

  size_t filtered_points = getPointCount(modifier.getOutputCloud());
  EXPECT_EQ(filtered_points, 125) << "After cropBox, should have 125 points (5x5x5)";
  EXPECT_LT(filtered_points, initial_points) << "Filtered cloud should have fewer points";

  // Test with invalid parameters (too few)
  size_t before_invalid = getPointCount(modifier.getOutputCloud());
  std::vector<double> invalid_params = {1.0, 2.0};  // Only 2 params, need 6
  modifier.cropBox(invalid_params);
  size_t after_invalid = getPointCount(modifier.getOutputCloud());
  EXPECT_EQ(before_invalid, after_invalid) << "Invalid parameters should not change point count";
}
/**
 * @brief Test cropSphere filter
 */
TEST(pointcloudmodifier, cropSphere)
{
  pointcloudmodifierlib::Modifier modifier;
  auto test_cloud = createTestPointCloud();
  modifier.setCloud(test_cloud);

  size_t initial_points = getPointCount(modifier.getOutputCloud());
  EXPECT_EQ(initial_points, 1331) << "Initial point count should be 1331";

  // Crop to sphere with radius 2.5 (should include points within distance 2.5 from origin)
  // Points at distance <= 2.5: approximately points within sphere of radius 2.5
  // This will include points like (0,0,0), (1,0,0), (2,0,0), etc.
  double radius = 2.5;
  modifier.cropSphere(radius);

  size_t filtered_points = getPointCount(modifier.getOutputCloud());
  EXPECT_LT(filtered_points, initial_points) << "Filtered cloud should have fewer points";
  EXPECT_GT(filtered_points, 0) << "Filtered cloud should have some points";

  // With a very small radius, should have very few points
  modifier.setCloud(test_cloud);
  modifier.cropSphere(0.5);
  size_t small_radius_points = getPointCount(modifier.getOutputCloud());
  EXPECT_LT(small_radius_points, filtered_points) << "Smaller radius should have fewer points";
}
/**
 * @brief Test cropCylinder filter
 */
TEST(pointcloudmodifier, cropCylinder)
{
  pointcloudmodifierlib::Modifier modifier;
  auto test_cloud = createTestPointCloud();
  modifier.setCloud(test_cloud);

  size_t initial_points = getPointCount(modifier.getOutputCloud());
  EXPECT_EQ(initial_points, 1331) << "Initial point count should be 1331";

  // Crop to cylinder with radius 3.0 (points with sqrt(x^2 + y^2) < 3.0)
  // This should include many points near the z-axis
  double radius = 3.0;
  modifier.cropCylinder(radius);

  size_t filtered_points = getPointCount(modifier.getOutputCloud());
  EXPECT_LT(filtered_points, initial_points) << "Filtered cloud should have fewer points";
  EXPECT_GT(filtered_points, 0) << "Filtered cloud should have some points";

  // With a very small radius, should have fewer points
  modifier.setCloud(test_cloud);
  modifier.cropCylinder(1.0);
  size_t small_radius_points = getPointCount(modifier.getOutputCloud());
  EXPECT_LT(small_radius_points, filtered_points) << "Smaller radius should have fewer points";
}
/**
 * @brief Test voxelFilter
 */
TEST(pointcloudmodifier, voxelFilter)
{
  pointcloudmodifierlib::Modifier modifier;
  auto test_cloud = createTestPointCloud();
  modifier.setCloud(test_cloud);

  size_t initial_points = getPointCount(modifier.getOutputCloud());
  EXPECT_EQ(initial_points, 1331) << "Initial point count should be 1331";

  // Apply voxel filter with leaf size 2.0 (should downsample significantly)
  std::vector<double> voxel_size = {2.0, 2.0, 2.0};
  modifier.voxelFilter(voxel_size);

  size_t filtered_points = getPointCount(modifier.getOutputCloud());
  EXPECT_LT(filtered_points, initial_points) << "Voxel filter should reduce point count";
  EXPECT_GT(filtered_points, 0) << "Voxel filter should keep some points";

  // With larger voxel size, should have even fewer points
  modifier.setCloud(test_cloud);
  std::vector<double> large_voxel = {5.0, 5.0, 5.0};
  modifier.voxelFilter(large_voxel);
  size_t large_voxel_points = getPointCount(modifier.getOutputCloud());
  EXPECT_LT(large_voxel_points, filtered_points) << "Larger voxel size should have fewer points";

  // Test with invalid parameters (too few)
  modifier.setCloud(test_cloud);
  size_t before_invalid = getPointCount(modifier.getOutputCloud());
  std::vector<double> invalid_voxel = {1.0};  // Only 1 param, need 3
  modifier.voxelFilter(invalid_voxel);
  size_t after_invalid = getPointCount(modifier.getOutputCloud());
  EXPECT_EQ(before_invalid, after_invalid) << "Invalid parameters should not change point count";
}
/**
 * @brief Test outlierRadiusFilter
 */
TEST(pointcloudmodifier, outlierRadiusFilter)
{
  pointcloudmodifierlib::Modifier modifier;
  auto test_cloud = createTestPointCloud();
  modifier.setCloud(test_cloud);

  size_t initial_points = getPointCount(modifier.getOutputCloud());
  EXPECT_EQ(initial_points, 1331) << "Initial point count should be 1331";

  // Apply radius outlier removal with radius 0.5 and min neighbors 5
  // Since our grid has points at integer coordinates, most points should have neighbors
  double radius = 1.5;
  int min_neighbors = 5;
  modifier.outlierRadiusFilter(radius, min_neighbors);

  size_t filtered_points = getPointCount(modifier.getOutputCloud());
  // With a dense grid, most points should remain
  EXPECT_GT(filtered_points, 0) << "Filtered cloud should have some points";

  // With very strict parameters, should remove more points
  modifier.setCloud(test_cloud);
  modifier.outlierRadiusFilter(1.5, 10);
  size_t strict_filtered = getPointCount(modifier.getOutputCloud());
  EXPECT_LE(strict_filtered, filtered_points) << "Stricter filter should remove more points";
}
/**
 * @brief Test outlierStatFilter
 */
TEST(pointcloudmodifier, outlierStatFilter)
{
  pointcloudmodifierlib::Modifier modifier;
  auto test_cloud = createTestPointCloud();
  modifier.setCloud(test_cloud);

  size_t initial_points = getPointCount(modifier.getOutputCloud());
  EXPECT_EQ(initial_points, 1331) << "Initial point count should be 1331";

  // Apply statistical outlier removal
  // Mean K = 50, standard deviation multiplier = 1.0
  double threshold = 1.0;
  int mean_k = 50;
  modifier.outlierStatFilter(threshold, mean_k);

  size_t filtered_points = getPointCount(modifier.getOutputCloud());
  EXPECT_GT(filtered_points, 0) << "Filtered cloud should have some points";
  // With a regular grid, most points should remain
  EXPECT_GT(filtered_points, initial_points * 0.8) << "Regular grid should keep most points";

  // With stricter threshold, should remove more points
  modifier.setCloud(test_cloud);
  modifier.outlierStatFilter(0.5, mean_k);
  size_t strict_filtered = getPointCount(modifier.getOutputCloud());
  EXPECT_LE(strict_filtered, filtered_points) << "Stricter threshold should remove more points";
}
/**
 * @brief Test filter chaining
 */
TEST(pointcloudmodifier, filterChaining)
{
  pointcloudmodifierlib::Modifier modifier;
  auto test_cloud = createTestPointCloud();
  modifier.setCloud(test_cloud);

  size_t initial_points = getPointCount(modifier.getOutputCloud());
  EXPECT_EQ(initial_points, 1331) << "Initial point count should be 1331";

  // Chain multiple filters
  std::vector<double> box_params = {-3.0, -3.0, -3.0, 3.0, 3.0, 3.0};
  std::vector<double> voxel_size = {1.5, 1.5, 1.5};

  modifier.cropBox(box_params).voxelFilter(voxel_size);

  size_t filtered_points = getPointCount(modifier.getOutputCloud());
  EXPECT_LT(filtered_points, initial_points) << "Chained filters should reduce point count";
  EXPECT_GT(filtered_points, 0) << "Chained filters should keep some points";

  // Verify input cloud is unchanged
  size_t input_points = getPointCount(modifier.getInputCloud());
  EXPECT_EQ(input_points, initial_points) << "Input cloud should remain unchanged";
}
