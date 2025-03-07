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

namespace pointcloudmodifyer {

class PcdModifyer {
public:
    PcdModifyer();
    ~PcdModifyer();

    // Loading/saving functions
    bool loadPCD(const std::string& file_path);
    bool savePCD(const std::string& file_path);
       
    // Filter functions - all return reference to allow chaining
    PcdModifyer& cropBox(const std::vector<double>& box_params);
    PcdModifyer& cropSphere(const double& sphere_params);

    
    // Visualization
    void visualize();
    
    // Getters
    const pcl::PCLPointCloud2::Ptr getInputCloud() const { return input_cloud; }
    const pcl::PCLPointCloud2::Ptr getOutputCloud() const { return output_cloud; }

private:
    pcl::PCLPointCloud2::Ptr input_cloud;
    pcl::PCLPointCloud2::Ptr output_cloud;
    pcl::PCLPointCloud2::Ptr mod_cloud;
};

}  // namespace pointcloudmodifyer