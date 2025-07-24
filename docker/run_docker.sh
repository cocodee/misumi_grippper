#!/bin/bash
IMAGE_NAME="misumi_base:ros2"

docker run -it -v $(pwd):/misumi_gripper $IMAGE_NAME /bin/bash