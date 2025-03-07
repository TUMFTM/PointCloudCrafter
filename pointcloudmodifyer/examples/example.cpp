#include "pointcloudmodifyer.hpp"
#include <iostream>

int main(int argc, char *argv[]) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <input_pcd_file>" << std::endl;
        return 1;
    }

    std::string input_file = argv[1];
    pointcloudmodifyer::Modifyer modifier;
    
    // Load the PCD file
    if (!modifier.loadPCD(input_file)) {
        return 1;
    }

    // Example of applying multiple filters using the fluent interface
    modifier.voxelfilter({0.01, 0.01, 0.01}).cropBox({-30.0, -30.0, -30.0, 30.0, 30.0, 30.0});

    auto out = modifier.getOutputCloud();
    modifier.setCloud(out);

    modifier.cropSphere(10.0);

    auto check = modifier.getOutputCloud();
    std::cout << "Output cloud size: " << check->width * check->height << std::endl;

    // Save the result
    std::string output_file = "output.pcd";
    if (!modifier.savePCD(output_file)) {
        return 1;
    }
    
    // Visualize the result
    modifier.visualize();
    
    return 0;
}