#!/bin/bash
# Build script - checks for base image first

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"

# Restore base image if missing
if [[ -z "$(docker images -q isaac_ros_dev-aarch64:r36.4.4_ir3.2_frozen)" ]]; then
    echo "⚠️  Base image missing, restoring from backup..."
    cd "$SCRIPT_DIR"
    ./restore-base-image.sh
fi

IMAGE_NAME="cleaningrobot-vision"

echo "🔨 Building frozen container image..."
cd "$WORKSPACE_ROOT"
docker build -f src/isaac_ros_object_detection/isaac_ros_yolov8/myscripts/Dockerfile.production.v2 -t ${IMAGE_NAME}:latest .
echo ""
echo "✅ Build complete! Image: ${IMAGE_NAME}:latest"
echo ""
echo "Run with: ./deploy.sh"
