#include "pointcloudmodifyer.hpp"

#include <math.h>
#include <iostream>
#include <filesystem>
#include <limits>

// PCL
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/cloud_iterator.h>
#include <pcl/filters/extract_indices.h>

using Point = pcl::PCLPointCloud2;

namespace pointcloudmodifyer {

PcdModifyer::PcdModifyer()
    : input_cloud(new pcl::PCLPointCloud2()),
    output_cloud(new pcl::PCLPointCloud2()),
    mod_cloud(new pcl::PCLPointCloud2()) {}

PcdModifyer::~PcdModifyer() = default;

bool PcdModifyer::loadPCD(const std::string& file_path) {
    pcl::PCDReader reader;
    reader.read(file_path, *input_cloud);
    
    // Copy input to mod_cloud for processing
    *mod_cloud = *input_cloud;
    std::cout << "Loading completed: " << input_cloud->width * input_cloud->height << " data points" << std::endl;
    return true;
}

bool PcdModifyer::savePCD(const std::string& file_path) {
    std::string path = file_path;
    
    // If file_path is a directory, create a filename in that directory
    if (std::filesystem::is_directory(file_path)) {
        path = file_path + "/mod_cloud.pcd";
    }
    
    if (pcl::io::savePCDFile(path, *mod_cloud) == -1) {
        std::cerr << "Failed to save PCD file: " << path << std::endl;
        return false;
    }
    
    std::cout << "PCD file saved to: " << path << std::endl;
    return true;
}

void PcdModifyer::visualize() {
    // Convert to PointXYZ for visualization
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_xyz(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_xyz(new pcl::PointCloud<pcl::PointXYZ>());
    
    pcl::fromPCLPointCloud2(*input_cloud, *input_xyz);
    pcl::fromPCLPointCloud2(*mod_cloud, *output_xyz);
    
    pcl::visualization::PCLVisualizer viewer("PCD Viewer");
    
    // Add input cloud with white color
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> input_color(input_xyz, 255, 255, 255);
    viewer.addPointCloud(input_xyz, input_color, "input_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "input_cloud");
    
    // Add output cloud with red color
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> output_color(output_xyz, 230, 20, 20);
    viewer.addPointCloud(output_xyz, output_color, "output_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "output_cloud");
    
    // Additional visualization settings
    viewer.addCoordinateSystem(1.0, "cloud", 0);
    viewer.setBackgroundColor(0.15, 0.15, 0.15, 0);
    
    std::cout << "Input_cloud: white | Output_cloud: red" << std::endl;
    
    // Spin until window is closed
    viewer.spin();
}

PcdModifyer& PcdModifyer::cropBox(const std::vector<double>& box_params) {
    if (box_params.size() < 6) {
        std::cerr << "Error: cropBox requires 6 parameters (min_x, min_y, min_z, max_x, max_y, max_z)" << std::endl;
        return *this;
    }
       
    pcl::CropBox<Point> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(box_params[0], box_params[1], box_params[2], 1.0));
    boxFilter.setMax(Eigen::Vector4f(box_params[3], box_params[4], box_params[5], 1.0));
    boxFilter.setInputCloud(mod_cloud);
    boxFilter.filter(*mod_cloud);
    return *this;
}

PcdModifyer& PcdModifyer::cropSphere(const double& sphere_params) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*mod_cloud, *tmp);

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    
    for (size_t i = 0; i < tmp->size(); ++i) {
        if (tmp->points[i].getVector3fMap().norm() < sphere_params) {
            inliers->indices.push_back(i);
        }
    }
    
    pcl::ExtractIndices<pcl::PCLPointCloud2> extract;
    extract.setInputCloud(mod_cloud);
    extract.setIndices(inliers);
    extract.filter(*mod_cloud);
    
    return *this;
}

}  // namespace pointcloudmodifyer