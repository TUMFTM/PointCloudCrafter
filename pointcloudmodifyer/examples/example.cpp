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

#include <iostream>
#include <string>

#include "pointcloudmodifyer.hpp"
int main(int argc, char * argv[])
{
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
  modifier.outlierStatFilter(0.5, 25);

  auto out = modifier.getOutputCloud();
  modifier.setCloud(out);

  // modifier.cropSphere(10.0).timestampAnalyzer("time.txt");

  auto check = modifier.getOutputCloud();
  std::cout << "Output cloud size: " << check->width * check->height << std::endl;

  // Save the result
  if (!modifier.savePCD(input_file + "_mod")) {
    return 1;
  }

  // Visualize the result
  modifier.visualize();

  return 0;
}
