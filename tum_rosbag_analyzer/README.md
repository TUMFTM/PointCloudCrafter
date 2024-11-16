# tum_rosbag_analyzer

Package to analyze various signals from rosbags and compute statistical values etc.

## How to run

The general command to run this tool is:

```bash
ros2 run tum_rosbag_analyzer rosbag_analyzer <mode> /path/to/rosbag /topic/name
```

## Implemented modes

### Timestamps

Analyze the timestamps of your favorite PointCloud2 message ❤️.
- Extract the timestamp depending on its datatype and return information about the timestamps of the single points
and the header.
  1. Find out how the timestamps are computed and stored in the the pointcloud2 message.
  2. Check the difference between the smallest and largest timestamps to check the frequency.
  3. Check the convention for the header timestamp (e.g. first points stamp, meadian of timestamps, ...)