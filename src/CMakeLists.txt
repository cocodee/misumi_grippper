# CMake 最低版本要求
cmake_minimum_required(VERSION 3.10)

# 项目名称和 C++ 标准
project(misumi_gripper_controller VERSION 1.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 告诉 CMake 在我们自己的 cmake 目录中查找模块
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
# --- 查找依赖库 ---
# 查找 libmodbus 库
# find_package 是现代 CMake 查找库的标准方式
find_package(libmodbus REQUIRED)

# --- 定义库 ---
# 将我们的夹爪控制逻辑编译成一个静态库 (misumi_gripper_lib)
add_library(misumi_gripper_lib STATIC
    MisumiGripper.cpp
)

# 为我们的库指定头文件目录，这样其他目标可以轻松找到它
# PUBLIC 表示使用这个库的目标也会自动包含这个目录
target_include_directories(misumi_gripper_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 将 misumi_gripper_lib 链接到 libmodbus
# PUBLIC 表示链接关系会传递，如果其他目标链接了 misumi_gripper_lib，
# 它们也会自动链接到 libmodbus
target_link_libraries(misumi_gripper_lib PUBLIC
    libmodbus::modbus
)

# --- 定义可执行文件 ---
# 创建我们的主程序可执行文件
add_executable(run_gripper
    main.cpp
)

# 将主程序链接到我们自己创建的夹爪库
target_link_libraries(run_gripper PRIVATE
    misumi_gripper_lib
)

# --- 安装规则 (可选，但推荐) ---
# 这部分定义了 `make install` 命令的行为
# 它会将编译好的库、可执行文件和头文件安装到指定的系统目录
install(TARGETS run_gripper DESTINATION bin)
install(TARGETS misumi_gripper_lib DESTINATION lib)
install(FILES MisumiGripper.h DESTINATION include)

# --- 打印信息 ---
# 在配置时打印找到的 libmodbus 版本，方便调试
message(STATUS "Found libmodbus version: ${libmodbus_VERSION}")
message(STATUS "Project configured. You can now build with 'cmake --build .'")