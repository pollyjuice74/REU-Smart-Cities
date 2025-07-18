#!/bin/bash

CONTAINER_NAME=ros_lidar_fusion_env
IMAGE_NAME=ros:noetic
LOCAL_PROJECT=~/REU-Smart-Cities

echo "[+] Starting or creating ROS container..."

# Start or create container
docker ps -a --format '{{.Names}}' | grep -q "$CONTAINER_NAME"
if [ $? -eq 0 ]; then
    echo "[*] Container exists. Starting..."
    docker start -ai $CONTAINER_NAME
else
    echo "[*] Creating new container..."
    xhost +local:root
    docker run -it --name $CONTAINER_NAME \
        --net=host \
        --privileged \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -e DISPLAY=$DISPLAY \
        -v $LOCAL_PROJECT:/root/catkin_ws/src \
        $IMAGE_NAME bash -c '
            echo "[+] Updating and installing ROS tools..."
            apt update && apt install -y git nano wget python3-pip \
                ros-noetic-catkin python3-catkin-tools \
                ros-noetic-stage-ros ros-noetic-ira-laser-tools

            echo "[+] Creating catkin workspace..."
            mkdir -p /root/catkin_ws/src && cd /root/catkin_ws
            catkin_make
            echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc
            source /root/catkin_ws/devel/setup.bash

            echo "[+] Cloning ira_laser_tools..."
            cd /root/catkin_ws/src
            git clone https://github.com/iralabdisco/ira_laser_tools.git

            cd /root/catkin_ws
            catkin_make

            echo "[+] Creating example launch file..."
            cat <<EOF > /root/catkin_ws/src/merge_lidar.launch
<launch>
  <node pkg="ira_laser_tools" type="scan_to_scan_filter_chain" name="scan1_filter">
    <param name="scan_topic" value="/scan1"/>
    <param name="filtered_scan_topic" value="/filtered_scan1"/>
  </node>
</launch>
EOF

            echo "[✓] Setup complete. Ready to launch ROS nodes!"
            exec bash
        '
fi
