#!/bin/bash
IMAGE_NAME="misumi_base:ros2"

docker run -it \
    -v "$(pwd):/misumi_gripper" \
    -v "$(pwd)/../supre_robot_control/lib:/mylib" \
    $IMAGE_NAME \
    /bin/bash