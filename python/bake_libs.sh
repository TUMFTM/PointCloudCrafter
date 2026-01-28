#!/bin/bash

PREFIX=$1
DEST_LIB="$PREFIX/lib"
DEST_SHARE="$PREFIX/share"
ROS_PATH="/opt/ros/humble"

# Manual copy plugins, which are not detected by auditwheel
# TODO(ga58lar): find a way to automate this
cp /opt/ros/humble/lib/librmw_fastrtps_*.so* "$DEST_LIB/"
cp /opt/ros/humble/lib/librosbag2_storage_*.so* "$DEST_LIB/"
cp /opt/ros/humble/lib/libfast*.so* "$DEST_LIB/"

# Copy full ament index
if [ -d "$ROS_PATH/share/ament_index" ]; then
    cp -rp "$ROS_PATH/share/ament_index/." "$DEST_SHARE/ament_index/"
fi

# Copy share of required packages (package.xml)
PARENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )"/.. &> /dev/null && pwd )"
PKGS=($(grep -E '<(depend|exec_depend)>' "$PARENT_DIR/package.xml" | \
    sed -E 's/.*<(depend|exec_depend)>([^<]+)<\/(depend|exec_depend)>.*/\2/' | sort -u))

for pkg in "${PKGS[@]}"; do
    cp -rp /opt/ros/humble/share/$pkg "$DEST_SHARE/"
done