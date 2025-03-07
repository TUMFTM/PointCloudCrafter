#pragma once

#include <string>
#include <tuple>
#include <vector>
#include <memory>

// PCL
#define PCL_NO_PRECOMPILE
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>

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
    void setCloud(const PointCloud::Ptr& cloud) { *output_cloud = *cloud; *input_cloud = *cloud; };
    
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