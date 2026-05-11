<div align="center">
    <img src="docs/assets/logo.svg" alt="pointcloudcrafter" width="600" style="margin-bottom: 30px;">
</div>

<div align="center">

A toolkit for extracting, manipulating, and evaluating point clouds and 3D spatial maps. Includes functions for processing, analyzing, and visualizing point clouds, designed to streamline workflows in 3D mapping and general point cloud handling. Ideal for researchers and developers working with LiDAR, SLAM, and 3D spatial data.

[![Linux](https://img.shields.io/badge/os-linux-blue.svg)](https://www.linux.org/)
[![Docker](https://badgen.net/badge/icon/docker?icon=docker&label)](https://www.docker.com/)
[![ROS2jazzy](https://img.shields.io/badge/ros2-jazzy-blue.svg)](https://docs.ros.org/en/jazzy/index.html)
[![PyPI](https://github.com/TUMFTM/PointCloudCrafter/actions/workflows/pypi.yml/badge.svg)](https://github.com/TUMFTM/PointCloudCrafter/actions/workflows/pypi.yml)
[![docs](https://github.com/TUMFTM/PointCloudCrafter/actions/workflows/docs.yml/badge.svg)](https://github.com/TUMFTM/PointCloudCrafter/actions/workflows/docs.yml)
![Tested on](https://img.shields.io/badge/Tested%20on-Ubuntu%2022.04%20%7C%2024.04-E95420?logo=ubuntu&logoColor=white)
[![status](https://joss.theoj.org/papers/aaeee985344bdd07edb8119350bc69de/status.svg)](https://joss.theoj.org/papers/aaeee985344bdd07edb8119350bc69de)
</div>

<h2> Install </h2>

```bash
    pip install pointcloudcrafter
```

<h2> Usage </h2>

We provide a standalone Pip package, which is self-contained, so you do not have to worry about any dependencies and possible conflicts. We also provide the tool as ROS2 package. Both feature the full functionality, so you can decide what suits your needs best.

For rosbag-processing:

```bash
    pointcloudcrafter-rosbag -h

    ros2 run pointcloudcrafter rosbag -h
```

For file-processing:

```bash
    pointcloudcrafter-file -h

    ros2 run pointcloudcrafter file -h
```

<h2> Documentation</h2>

For more details on the features and how to use them, take a look at the [documentation](https://TUMFTM.github.io/PointCloudCrafter) hosted on GitHub Pages:  
[**https://TUMFTM.github.io/PointCloudCrafter**](https://TUMFTM.github.io/PointCloudCrafter)

<h2> Test Data </h2>

A small ROS 2 bag and example transforms are published as a pinned release
asset on this repository. To download the current test data:

```bash
curl -L https://github.com/TUMFTM/PointCloudCrafter/releases/download/testdata-v1/test-data.tar.gz \
    | tar xz -C /path/to/pointcloudcrafter-root/
```

The same asset is used by the CI test job, so you get exactly the data the
project is validated against. When the test data is updated, the tag is bumped
(e.g. `testdata-v2`).

## Contact

[Dominik Kulmer](mailto:dominik.kulmer@tum.de)  
[Maximilian Leitenstern](mailto:maxi.leitenstern@tum.de)  
Institute of Automotive Technology, School of Engineering and Design, Technical University of Munich, 85748 Garching, Germany