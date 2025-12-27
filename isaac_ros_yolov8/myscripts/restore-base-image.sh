#!/bin/bash
# Restore frozen base image if it's missing

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

if [[ -z "$(docker images -q isaac_ros_dev-aarch64:r36.4.4_ir3.2_frozen)" ]]; then
    echo "🔄 Restoring frozen base image..."
    gunzip -c isaac_ros_dev-aarch64-frozen.tar.gz | docker load
    echo "✅ Base image restored"
else
    echo "✅ Base image already exists"
fi
