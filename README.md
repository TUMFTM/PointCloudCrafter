<div align="center">
    <img src="docs/assets/logo.svg" alt="pointcloudcrafter" width="600" style="margin-bottom: 30px;">
</div>

<div align="center">

A toolkit for extracting, manipulating, and evaluating point clouds and 3D spatial maps. Includes functions for processing, analyzing, and visualizing point clouds, designed to streamline workflows in 3D mapping and general point cloud handling. Ideal for researchers and developers working with LiDAR, SLAM, and 3D spatial data.

[![Linux](https://img.shields.io/badge/os-linux-blue.svg)](https://www.linux.org/)
[![Docker](https://badgen.net/badge/icon/docker?icon=docker&label)](https://www.docker.com/)
[![ROS2humble](https://img.shields.io/badge/ros2-humble-blue.svg)](https://docs.ros.org/en/humble/index.html)
[![build_test](https://github.com/TUMFTM/PointCloudCrafter/actions/workflows/build_test.yml/badge.svg)](https://github.com/TUMFTM/PointCloudCrafter/actions/workflows/build_test.yml)
[![docs](https://github.com/TUMFTM/PointCloudCrafter/actions/workflows/docs.yml/badge.svg)](https://github.com/TUMFTM/PointCloudCrafter/actions/workflows/docs.yml)
</div>

<h2> Install </h2>

```bash
    pip install pointcloudcrafter
```
Next, follow the instructions on how to run the system by typing:

```bash
    pointcloudcrafter-rosbag -h
```

or:
```bash
    pointcloudcrafter-file -h
```

<h2> Documentation</h2>

Github Pages: [https://TUMFTM.github.io/PointCloudCrafter](https://TUMFTM.github.io/PointCloudCrafter)

<h2> Cloning without test data (recommended) </h2>

This repository uses Git LFS for large test files (22 MB).
Most users do not need them.

Clone without downloading LFS files:

```bash
    GIT_LFS_SKIP_SMUDGE=1 git clone https://github.com/TUMFTM/PointCloudCrafter.git
```

To download them later:

```bash
    git lfs pull
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

For more details on functionalities and usage, see the [documentation](https://TUMFTM.github.io/PointCloudCrafter)

## Contact

[Dominik Kulmer](mailto:dominik.kulmer@tum.de)  
[Maximilian Leitenstern](mailto:maxi.leitenstern@tum.de)  
Institute of Automotive Technology, School of Engineering and Design, Technical University of Munich, 85748 Garching, Germany