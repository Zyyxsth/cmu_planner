#!/bin/bash
# 构建完整的 M-TARE Docker 镜像（包含代码和编译）

echo "=================================="
echo "Building Complete M-TARE Image"
echo "=================================="
echo "This will:"
echo "  1. Download ROS2 Humble base image"
echo "  2. Clone code from GitHub"
echo "  3. Install all dependencies"
echo "  4. Compile all packages"
echo ""
echo "This will take 15-30 minutes..."
echo ""

sudo docker build -f Dockerfile.complete -t mtare-complete:latest .

echo ""
echo "=================================="
echo "Build complete!"
echo "Image: mtare-complete:latest"
echo "=================================="
