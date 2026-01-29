#!/bin/bash

PREFIX=$1
DEST_LIB="$PREFIX/lib"
DEST_SHARE="$PREFIX/share"
ROS_PATH="/opt/ros/humble"

# TODO(ga58lar): find a way to automate this
# Manual copy plugins, which are not detected by auditwheel
cp /opt/ros/humble/lib/librmw_fastrtps_*.so* "$DEST_LIB/"
cp /opt/ros/humble/lib/librosbag2_storage_default_plugins.so* "$DEST_LIB/"
cp /opt/ros/humble/lib/librosbag2_storage_mcap.so* "$DEST_LIB/"
cp /opt/ros/humble/lib/libfast*.so* "$DEST_LIB/"

# Copy full ament index
if [ -d "$ROS_PATH/share/ament_index" ]; then
    cp -rp "$ROS_PATH/share/ament_index/." "$DEST_SHARE/ament_index/"
fi

# Copy share of required packages (package.xml)
PARENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )"/.. &> /dev/null && pwd )"
PKGS=($(grep -E '<(depend|exec_depend)>' "$PARENT_DIR/package.xml" | \
    sed -E 's/.*<(depend|exec_depend)>([^<]+)<\/(depend|exec_depend)>.*/\2/' | sort -u))
EXTRA_PKGS=(
    "rmw" "rmw_implementation" "rcl" "rclcpp" "fastrtps" "fastcdr"
    "std_msgs" "builtin_interfaces" "geometry_msgs" "action_msgs" "unique_identifier_msgs"
    "rcl_interfaces" "rosgraph_msgs" "statistics_msgs" "pcl_msgs" "tf2_msgs" "tf2"
    "rosbag2_storage" "rosbag2_storage_default_plugins" "rosbag2_storage_mcap"
    "rosbag2_compression" "rosbag2_compression_zstd"
    "ament_index" "rcutils" "pluginlib"
    "rosidl_typesupport_cpp" "rosidl_typesupport_c"
    "rosidl_typesupport_fastrtps_c" "rosidl_typesupport_fastrtps_cpp"
    "rosidl_typesupport_introspection_c" "rosidl_typesupport_introspection_cpp"
)
ALL_PKGS=("${PKGS[@]}" "${EXTRA_PKGS[@]}")

for pkg in "${ALL_PKGS[@]}"; do
    if [ -d "$ROS_PATH/share/$pkg" ]; then
        cp -aL "$ROS_PATH/share/$pkg" "$DEST_SHARE/"
    fi

    find "$ROS_PATH/lib" \
    -name "lib${pkg}*.so*" \
    ! -name "*__rosidl_generator_py.so" \
    -exec cp -P {} "$DEST_LIB/" \;
done

mkdir -p "$DEST_SHARE/ament_index/resource_index/rmw_typesupport"
touch "$DEST_SHARE/ament_index/resource_index/rmw_typesupport/rmw_fastrtps_cpp"

# Clean up test plugins and data
find "$DEST_LIB" "$DEST_SHARE" -name "*.cmake" -delete
find "$DEST_SHARE" -name "*.pc" -delete
find "$DEST_SHARE" -name "*ConfigVersion.cmake" -delete
find "$DEST_SHARE" -name "*Targets.cmake" -delete