#!/bin/bash
# 构建完整 Docker 镜像（包含编译）

echo "Building M-TARE Docker image (this will take 15-30 minutes)..."
printf " " | sudo -S docker build -f Dockerfile.full -t mtare-planner:latest .

echo "Build complete!"
