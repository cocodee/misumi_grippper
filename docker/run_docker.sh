#!/bin/bash
IMAGE_NAME="misumi_base:latest"

docker run -it -v $(pwd):/misumi_gripper $IMAGE_NAME /bin/bash