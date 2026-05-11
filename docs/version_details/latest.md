# Latest

This page is regenerated on every documentation build from the
`ghcr.io/tumftm/pointcloudcrafter:latest` Docker image, so it always matches
`main`.

## Supported platforms

| Component        | Requirement                                              |
|------------------|----------------------------------------------------------|
| Operating system | Ubuntu 24.04 (Noble Numbat), x86_64                      |
| glibc            | >= 2.39 (wheel platform tag `manylinux_2_39_x86_64`)     |
| ROS 2 distro     | Jazzy Jalisco                                            |
| Python           | CPython 3.12                                             |
| Wheel tag        | `cp312-cp312-manylinux_2_39_x86_64`                      |
| Architectures    | `x86_64` only (no `aarch64` / `arm64` wheel)             |

For other platforms or architectures, use the
[Docker image](https://ghcr.io/tumftm/pointcloudcrafter) or build from source.

## `pointcloudcrafter-file`

CLI for processing point cloud files (PCD, PLY, TXT, KITTI, nuScenes).
Equivalent ROS 2 invocation: `ros2 run pointcloudcrafter file`.

```text
--8<-- "version_details/snippets/help_file_latest.txt"
```

## `pointcloudcrafter-rosbag`

CLI for extracting and processing point clouds from ROS 2 bag recordings.
Equivalent ROS 2 invocation: `ros2 run pointcloudcrafter rosbag`.

```text
--8<-- "version_details/snippets/help_rosbag_latest.txt"
```
