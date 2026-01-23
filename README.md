<div align="center">
    <img src="docs/logo.svg" alt="pointcloudcrafter" width="600" style="margin-bottom: 30px;">
</div>

<div align="center">

A toolkit for extracting, manipulating, and evaluating point clouds and 3D spatial maps. Includes functions for processing, analyzing, and visualizing point clouds, designed to streamline workflows in 3D mapping and general point cloud handling. Ideal for researchers and developers working with LiDAR, SLAM, and 3D spatial data.

[![Linux](https://img.shields.io/badge/os-linux-blue.svg)](https://www.linux.org/)
[![Docker](https://badgen.net/badge/icon/docker?icon=docker&label)](https://www.docker.com/)
[![ROS2humble](https://img.shields.io/badge/ros2-humble-blue.svg)](https://docs.ros.org/en/humble/index.html)
</div>

<h2>Docker Usage</h2>

Pull the latest version of the provided docker image from the github registry:

```bash
docker pull ghcr.io/tumftm/pointcloudcrafter:latest
```

If you want to build the docker image yourself, run the following script:

```bash
./docker/build_docker.sh
```

To run the docker container use:
```bash
./docker/run_docker.sh /path/to/data/directory
```

<br>

<h2> Usage and Command Line Arguments </h2>

<h3> Create and modify point clouds from rosbags </h3>

```
Usage: ros2 run pointcloudcrafter rosbag [OPTIONS] bag-path out-dir topic-names...

Positionals:
  bag-path TEXT REQUIRED      Path to ROS 2 bag
  out-dir TEXT REQUIRED       Output directory for .pcd files
  topic-names TEXT ... REQUIRED
                              PointCloud2 topic names

Options:
  -h,--help                   Print this help message and exit


Output:
  --timestamps                Save point cloud timestamps to a text file
  --sequential-name           Use sequential file names


Transforms:
  -t,--target-frame TEXT      Target TF frame for all point clouds
  --tf,--transform-file TEXT  TXT file with transform ([frame_id] r1 r2 r3 x r4 r5 r6 y r7 r8 r9 z)


General:
  -m,--max-frames INT         Maximum number of frames (-1 = unlimited)
  -j,--skip-frames INT        Number of frames to skip at the beginning
  -s,--stride-frames INT      Write every Nth frame


File Format:
  --save-pcd                  Save PCD files (default)
  --save-ply                  Save PLY files
  --save-txt                  Save TXT ASCII files
  --save-kitti                Save KITTI binary files
  --save-nuscenes             Save nuScenes binary files


Filtering:
  --cb,--crop-box FLOAT FLOAT FLOAT FLOAT FLOAT FLOAT x 6
                              Crop box [xmin ymin zmin xmax ymax zmax]
  --cs,--crop-sphere FLOAT    Crop to sphere with given radius
  --cc,--crop-cylinder FLOAT  Crop to cylinder with given radius
  --inverse-crop              Inverse crop filters
  --vf,--voxel-filter FLOAT FLOAT FLOAT x 3
                              Voxel size [x y z]
  --orf,--outlier-radius-filter FLOAT INT x 2
                              Radius outlier removal [radius min_neighbors]
  --osf,--outlier-stat-filter FLOAT INT x 2
                              Statistical outlier removal [threshold mean_k]


Example:
  ros2 run pointcloudcrafter rosbag /datasets/bag.mcap /datasets/out/ /points_raw 
    --voxel-filter 0.1 0.1 0.1 --stride-frames 5
```

<h3> Modify point clouds from files </h3>

```
Usage: ros2 run pointcloudcrafter file [OPTIONS] input-path out-dir

Positionals:
  input-path TEXT REQUIRED    Path to point cloud file or directory
  out-dir TEXT REQUIRED       Output directory for point cloud file(s)

Options:
  -h,--help                   Print this help message and exit


File Format:
  --load-pcd                  Load PCD files (default)
  --load-ply                  Load PLY files
  --load-txt                  Load TXT ASCII files
  --load-kitti                Load KITTI binary files
  --load-nuscenes             Load nuScenes binary files
  --load-obj                  Load OBJ files
  --save-pcd                  Save PCD files (default)
  --save-ply                  Save PLY files
  --save-txt                  Save TXT ASCII files
  --save-kitti                Save KITTI binary files
  --save-nuscenes             Save nuScenes binary files


Transforms:
  -t,--translation FLOAT FLOAT FLOAT x 3
                              Translation [x y z]
  -r,--rotation FLOAT FLOAT FLOAT x 3
                              Rotation [roll pitch yaw]
  --deg                       Rotation in degrees instead of radians
  --tf,--transform-file TEXT  TXT file with transform ([frame_id] r1 r2 r3 x r4 r5 r6 y r7 r8 r9 z)


Output:
  --sequential-name           Use sequential file names


General:
  -m,--max-frames INT         Maximum number of frames (-1 = unlimited)
  -j,--skip-frames INT        Number of frames to skip at the beginning
  -s,--stride-frames INT      Write every Nth frame


Filtering:
  --cb,--crop-box FLOAT FLOAT FLOAT FLOAT FLOAT FLOAT x 6
                              Crop box [xmin ymin zmin xmax ymax zmax]
  --cs,--crop-sphere FLOAT    Crop to sphere with given radius
  --cc,--crop-cylinder FLOAT  Crop to cylinder with given radius
  --inverse-crop              Inverse crop filters
  --vf,--voxel-filter FLOAT FLOAT FLOAT x 3
                              Voxel size [x y z]
  --orf,--outlier-radius-filter FLOAT INT x 2
                              Radius outlier removal [radius min_neighbors]
  --osf,--outlier-stat-filter FLOAT INT x 2
                              Statistical outlier removal [threshold mean_k]


Example:
  ros2 run pointcloudcrafter file /datasets/input/ /datasets/out/ 
    --voxel-filter 0.1 0.1 0.1 -m 5
```

<br>

<h2> Cloning without test data (recommended) </h2>

This repository uses Git LFS for large test files (22 MB).
Most users do not need them.

Clone without downloading LFS files:

    GIT_LFS_SKIP_SMUDGE=1 git clone https://github.com/TUMFTM/PointCloudCrafter.git

To download them later:

    git lfs pull

<br>

<h2> Third-Party Licenses </h2>

<details>
<summary>More information</summary>
<br>

This project uses the following third-party libraries:

| Library                                                      | License       |
| ------------------------------------------------------------ | ------------- |
| [CLI11](https://github.com/CLIUtils/CLI11)                   | BSD           |
| [Eigen](https://eigen.tuxfamily.org/)                        | MPL-2.0       |
| [fmt](https://github.com/fmtlib/fmt)                         | MIT           |
| [geometry_msgs](https://github.com/ros2/common_interfaces)   | Apache-2.0    |
| [message_filters](https://github.com/ros2/message_filters)   | BSD           |
| [PCL](https://pointclouds.org/)                              | BSD           |
| [pcl_conversions](https://github.com/ros-perception/perception_pcl)  | BSD   |
| [rclcpp](https://github.com/ros2/rclcpp)                     | Apache-2.0    |
| [rosbag2_cpp](https://github.com/ros2/rosbag2)               | Apache-2.0    |
| [sensor_msgs](https://github.com/ros2/common_interfaces)     | Apache-2.0    |
| [tf2_ros](https://github.com/ros2/geometry2)                 | BSD           |
| [tf2_msgs](https://github.com/ros2/geometry2)                | BSD           |

> **Note:** This list may not be exhaustive. Please refer to the individual package documentation for complete license information.

</details>

<h2> More Point Cloud Handling </h2>

<details>
<summary>More information</summary>
<br>

| Tool                                                              | Type            |
| ----------------------------------------------------------------- | --------------- |
| [CloudCompare](https://github.com/CloudCompare/CloudCompare)      | GUI & CLI       |
| [libLAS](https://liblas.org/)                                     | Library         |
| [libpointmatcher](https://github.com/norlab-ulaval/libpointmatcher) | Library       |
| [Open3D](https://www.open3d.org/)                                 | Library         |
| [PCL](https://github.com/PointCloudLibrary/pcl)                   | Library & CLI   |
| [PDAL](https://pdal.io/)                                          | Library & CLI   |
| [PointCloudEditor](https://github.com/JohannesKrueger/pointcloudeditor) | GUI       |

</details>