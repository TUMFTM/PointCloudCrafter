/*
 * Copyright (C) 2024 Dominik Kulmer, Maximilian Leitenstern
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

 #include "pointcloudmodifyer.hpp"
 #include <iostream>
 
 int main(int argc, char *argv[]) {
     if (argc < 2) {
         std::cout << "Usage: " << argv[0] << " <input_pcd_file>" << std::endl;
         return 1;
     }
 
     std::string input_file = argv[1];
     pointcloudmodifyer::PcdModifyer modifier;
     
     // Load the PCD file
     if (!modifier.loadPCD(input_file)) {
         return 1;
     }
     
     // Example of applying multiple filters using the fluent interface
     modifier.cropBox({-10.0, -10.0, -10.0, 10.0, 10.0, 10.0});
     
     // Save the result
     std::string output_file = "output.pcd";
     if (!modifier.savePCD(output_file)) {
         return 1;
     }
     
     // Visualize the result
     modifier.visualize();
     
     return 0;
 }