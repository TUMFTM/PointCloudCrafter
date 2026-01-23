# Usage and Command Line Arguments

## Create and modify point clouds from rosbags

```bash
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

## Modify point clouds from files

```bash
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
