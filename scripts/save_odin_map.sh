#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${REPO_ROOT}"

echo "Triggering ODIN map save..."
"${REPO_ROOT}/src/odin_ros_driver/set_param.sh" save_map 1
echo "If the driver is still running, the map will be written under:"
echo "  ${REPO_ROOT}/src/odin_ros_driver/map/manual_maps"
