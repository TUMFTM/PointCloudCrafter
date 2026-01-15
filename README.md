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

<h2> Usage and Command Line Arguments </h2>

```
Usage: ros2 run pointcloudcrafter crafter [OPTIONS] bag-path out-dir topic-names...

Positionals:
  bag-path TEXT REQUIRED      Path to ROS 2 bag
  out-dir TEXT REQUIRED       Output directory for .pcd files
  topic-names TEXT ... REQUIRED
                              PointCloud2 topic names

Options:
  -h,--help                   Print this help message and exit


Output:
  --sequential-name           Use sequential file names instead of timestamps
  --timestamps                Save point cloud timestamps to a text file


General:
  -m,--max-frames INT         Maximum number of frames to extract (-1 = unlimited)
  -j,--skip-frames INT        Number of frames to skip at the beginning
  -s,--stride-frames INT      Write every Nth frame


Transforms:
  -t,--target-frame TEXT      Target TF frame for all point clouds
  --transform-file,--tf TEXT  TXT file with additional transforms (frame_id r1 r2 r3 x r4 r5 r6 y r7 r8 r9 z)


Filtering:
  --crop-box,--cb FLOAT FLOAT FLOAT FLOAT FLOAT FLOAT x 6
                              Crop box [xmin ymin zmin xmax ymax zmax]
  --crop-sphere,--cs FLOAT    Crop to sphere with given radius
  --crop-cylinder,--cc FLOAT  Crop to cylinder with given radius
  --voxel-filter,--vf FLOAT FLOAT FLOAT x 3
                              Voxel size [x y z]
  --outlier-radius-filter,--orf FLOAT INT x 2
                              Radius outlier removal [radius min_neighbors]
  --outlier-stat-filter,--osf FLOAT INT x 2
                              Statistical outlier removal [threshold mean_k]


Example:
  ros2 run pointcloudcrafter crafter bag.mcap out/ /points_raw 
    --voxel-filter 0.1 0.1 0.1 --stride-frames 5
```
