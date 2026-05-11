#!/usr/bin/env bash
# Capture CLI help output and generate a Version Details page for one tag.
#
# Usage:
#   capture_version.sh <tag>
#
# <tag> is a Docker image tag of ghcr.io/tumftm/pointcloudcrafter, e.g. "latest"
# or "v1.4.2". The script introspects the image to determine the Ubuntu release,
# ROS 2 distro, Python version, and architecture, so older releases (e.g. those
# built on Ubuntu 22.04 / ROS 2 Humble) are documented correctly without any
# version-specific logic in this script.
#
# Writes:
#   docs/version_details/snippets/help_file_<tag>.txt
#   docs/version_details/snippets/help_rosbag_<tag>.txt
# For non-"latest" tags, also writes:
#   docs/version_details/<tag>.md
set -euo pipefail

TAG="${1:?usage: capture_version.sh <tag>}"
IMAGE="ghcr.io/tumftm/pointcloudcrafter:${TAG}"

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
SNIP_DIR="${ROOT}/docs/version_details/snippets"
PAGE_DIR="${ROOT}/docs/version_details"
mkdir -p "${SNIP_DIR}"

echo "Pulling ${IMAGE}..."
docker pull "${IMAGE}" >/dev/null

# Introspect the image: Ubuntu release, ROS distro, Python version, arch.
META=$(docker run --rm --entrypoint /bin/bash "${IMAGE}" -c '
    . /etc/os-release
    ROS_DISTRO=$(ls /opt/ros 2>/dev/null | head -n1)
    PY=$(python3 --version 2>&1 | awk "{print \$2}")
    ARCH=$(uname -m)
    GLIBC=$(ldd --version 2>/dev/null | head -n1 | awk "{print \$NF}")
    echo "UBUNTU_VERSION=${VERSION_ID}"
    echo "UBUNTU_NAME=${VERSION_CODENAME^}"
    echo "ROS_DISTRO=${ROS_DISTRO}"
    echo "PY_VERSION=${PY}"
    echo "ARCH=${ARCH}"
    echo "GLIBC=${GLIBC}"
')
eval "${META}"

PY_MAJMIN=$(echo "${PY_VERSION}" | cut -d. -f1,2)
PY_TAG="cp$(echo "${PY_MAJMIN}" | tr -d .)"

run_help() {
    local exe="$1"
    docker run --rm --entrypoint /bin/bash "${IMAGE}" \
        -c "source /ros_ws/install/setup.bash && ros2 run pointcloudcrafter ${exe} -h"
}

echo "Capturing help output (Ubuntu ${UBUNTU_VERSION}, ROS ${ROS_DISTRO}, Python ${PY_VERSION})..."
run_help file   > "${SNIP_DIR}/help_file_${TAG}.txt"
run_help rosbag > "${SNIP_DIR}/help_rosbag_${TAG}.txt"

if [[ "${TAG}" == "latest" ]]; then
    echo "Skipping page generation for 'latest' (page is committed)."
    exit 0
fi

PAGE="${PAGE_DIR}/${TAG}.md"
cat > "${PAGE}" <<EOF
# ${TAG}

Frozen snapshot captured from \`${IMAGE}\` at release time.

## Supported platforms

| Component        | Requirement                                              |
|------------------|----------------------------------------------------------|
| Operating system | Ubuntu ${UBUNTU_VERSION} (${UBUNTU_NAME}), ${ARCH}       |
| glibc            | ${GLIBC}                                                 |
| ROS 2 distro     | ${ROS_DISTRO^}                                           |
| Python           | CPython ${PY_VERSION}                                    |
| Wheel tag        | \`${PY_TAG}-${PY_TAG}-manylinux_*_${ARCH}\`              |
| Architectures    | \`${ARCH}\` only                                         |

## \`pointcloudcrafter-file\`

\`\`\`text
--8<-- "version_details/snippets/help_file_${TAG}.txt"
\`\`\`

## \`pointcloudcrafter-rosbag\`

\`\`\`text
--8<-- "version_details/snippets/help_rosbag_${TAG}.txt"
\`\`\`
EOF
echo "Wrote ${PAGE}"

# Insert the new version into the index table if not present.
INDEX="${PAGE_DIR}/index.md"
if ! grep -q "(${TAG}.md)" "${INDEX}"; then
    ROW="| ${TAG} | released | [${TAG}](${TAG}.md) |"
    awk -v row="${ROW}" '
        /<!-- VERSION_TABLE_INSERT -->/ { print row }
        { print }
    ' "${INDEX}" > "${INDEX}.new"
    mv "${INDEX}.new" "${INDEX}"
    echo "Added ${TAG} to ${INDEX}"
fi
