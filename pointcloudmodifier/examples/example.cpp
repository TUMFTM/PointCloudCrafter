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
#include <filesystem>  // NOLINT

#include "formats.hpp"
#include "pointcloudmodifier.hpp"
int main(int argc, char * argv[])
{
  if (argc < 2) {
    std::cout << "Usage: " << argv[0] << " <input_pcd_file>" << std::endl;
    return 1;
  }

  std::string input_file = argv[1];
  std::filesystem::path output_path(input_file);
  pointcloudmodifierlib::Modifier modifier;
  auto format = pointcloudcrafter::tools::formats::FileFormat::PCD;

  // Load the PCD file
  if (!modifier.load(input_file, format)) {
    return 1;
  }

  // Example of applying multiple filters using the fluent interface
  modifier.outlierStatFilter(0.5, 25);

  auto out = modifier.getOutputCloud();
  modifier.setCloud(out);

  modifier.cropSphere(10.0).timestampAnalyzer(output_path.parent_path().string() + "/time.txt");

  auto check = modifier.getOutputCloud();
  std::cout << "Output cloud size: " << check->width * check->height << std::endl;

  // Save the result

  output_path.replace_filename("modified_" + output_path.filename().string());
  if (!modifier.save(output_path.string(), format)) {
    return 1;
  }

  // Visualize the result
  modifier.visualize();

  return 0;
}
