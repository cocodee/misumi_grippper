# ===================================================================
# 阶段 1: 构建环境 (Builder Stage)
# 使用一个包含完整构建工具的镜像
# ===================================================================
FROM ubuntu:22.04

# 设置非交互式安装，避免apt-get卡住
ENV DEBIAN_FRONTEND=noninteractive

#RUN sed -i 's#http://archive.ubuntu.com/ubuntu/#http://mirrors.aliyun.com/ubuntu/#' /etc/apt/sources.list
# 安装构建依赖：编译器、CMake 和 libmodbus 开发包
RUN apt update && \
    apt install -y --no-install-recommends \
    g++ \
    cmake \
    libmodbus-dev \
    vim \
    && apt clean && rm -rf /var/lib/apt/lists/*

# 将项目文件复制到容器中
WORKDIR /app
COPY . .

# 运行 CMake 和 build
RUN cd src && cmake -S . -B build && \
    cmake --build build

# ===================================================================
# 阶段 2: 运行环境 (Runner Stage)
# 使用一个非常轻量的基础镜像，减小最终体积
# ===================================================================
#FROM ubutu:22.04 AS runner

# 安装运行时依赖：只需要 libmodbus 的运行时库
#RUN apt-get update && \
#    apt-get install -y --no-install-recommends \
#    libmodbus5 \
#    && apt-get clean && rm -rf /var/lib/apt/lists/*

# 设置工作目录
#WORKDIR /app

# 从构建阶段复制编译好的可执行文件
#COPY --from=builder /app/build/run_gripper .

# 设定容器启动时执行的默认命令
# 注意：你需要将 /dev/ttyUSB0 挂载到容器中才能真正与硬件通信
#CMD ["./run_gripper"]
#CMD ["./run_gripper"]
CMD ["/bin/bash"]