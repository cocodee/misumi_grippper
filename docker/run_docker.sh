#!/bin/bash
#IMAGE_NAME="eu_motor_base:latest"
IMAGE_NAME="eu_motor_base:latest"
docker run -it \
    -v "$(pwd):/misumi_gripper" \
    $IMAGE_NAME \
    /bin/bash