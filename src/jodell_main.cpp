// src/main.cpp
#include "GripperBus.hpp"
#include "JodellGrippper.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <limits> // 用于 std::numeric_limits

// --- 配置区 ---
// 在 Linux 上可能是 "/dev/ttyUSB0", 在 Windows 上是 "COM3" 等
const std::string SERIAL_PORT = "/dev/ttyUSB0"; 
const int SLAVE_ID = 9; // 夹爪的默认从站 ID
// --- 配置区结束 ---

// 辅助函数：等待用户按 Enter 键继续
void pressEnterToContinue() {
    std::cout << "\n... 按 Enter 键继续 ...\n" << std::endl;
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

int main() {
    std::cout << "========= Jodell EPG Gripper Library Test Suite =========\n" << std::endl;

    // --- 1. GripperBus 连接测试 ---
    std::cout << "--- [1] 测试 GripperBus 连接 ---" << std::endl;
    
    std::cout << "a) 尝试连接一个无效的串口 '/dev/nonexistent' (预期失败)..." << std::endl;
    GripperBus invalid_bus("/dev/nonexistent");
    if (!invalid_bus.connect()) {
        std::cout << "成功: 无效串口连接失败，符合预期。\n" << std::endl;
    } else {
        std::cerr << "失败: 竟然成功连接到了一个无效的串口！\n" << std::endl;
        invalid_bus.disconnect();
    }

    std::cout << "b) 尝试连接到配置的串口: " << SERIAL_PORT << " ..." << std::endl;
    GripperBus bus(SERIAL_PORT);
    if (!bus.connect()) {
        std::cerr << "错误: 无法连接到串口 " << SERIAL_PORT << ". 请检查端口名和权限。" << std::endl;
        return -1;
    }
    std::cout << "成功: 串口连接成功。\n" << std::endl;

    // --- 2. Gripper 初始化和使能测试 ---
    std::cout << "--- [2] 测试 Gripper 初始化和使能 ---" << std::endl;
    JodellGripper gripper(bus, SLAVE_ID);
    GripperStatus status;

    std::cout << "a) 使能 (激活) 夹爪 ID: " << SLAVE_ID << "..." << std::endl;
    if (!gripper.enable()) {
        std::cerr << "错误: 使能夹爪失败。" << std::endl;
        bus.disconnect();
        return -1;
    }
    
    std::cout << "夹爪正在进行初始化动作，请稍候..." << std::endl;
    if (!gripper.waitMotionComplete(10000)) { // 增加超时时间以防初始化较慢
        std::cerr << "警告: 等待初始化完成超时。" << std::endl;
    } else {
        std::cout << "成功: 夹爪初始化完成。\n" << std::endl;
    }

    std::cout << "b) 获取夹爪初始化后的状态..." << std::endl;
    if (gripper.getStatus(status)) {
        status.print();
    } else {
        std::cerr << "错误: 获取状态失败。\n" << std::endl;
    }

    pressEnterToContinue();

    // --- 3. Gripper 移动功能测试 ---
    std::cout << "--- [3] 测试 Gripper 移动功能 (move) ---" << std::endl;

    std::cout << "a) 指令: 全速全力打开 (位置=0, 速度=255, 力=255)..." << std::endl;
    if (!gripper.move(0, 255, 255)) {
        std::cerr << "错误: 发送打开指令失败。" << std::endl;
    } else {
        gripper.waitMotionComplete();
        std::cout << "移动完成。当前状态:" << std::endl;
        gripper.getStatus(status);
        status.print();
    }

    pressEnterToContinue();

    std::cout << "b) 指令: 半速半力闭合 (位置=255, 速度=128, 力=128)..." << std::endl;
    if (!gripper.move(255, 128, 128)) {
        std::cerr << "错误: 发送闭合指令失败。" << std::endl;
    } else {
        gripper.waitMotionComplete();
        std::cout << "移动完成。当前状态:" << std::endl;
        gripper.getStatus(status);
        status.print();
    }

    pressEnterToContinue();

    std::cout << "c) 指令: 低速移动到中间位置 (位置=150, 速度=80, 力=200)..." << std::endl;
    if (!gripper.move(150, 80, 200)) {
        std::cerr << "错误: 发送移动指令失败。" << std::endl;
    } else {
        gripper.waitMotionComplete();
        std::cout << "移动完成。当前状态:" << std::endl;
        gripper.getStatus(status);
        status.print();
    }
    
    // --- 4. 夹取物体测试 (手动) ---
    std::cout << "\n--- [4] 测试物体检测 (手动操作) ---" << std::endl;
    std::cout << "请在夹爪路径中放置一个物体。" << std::endl;
    pressEnterToContinue();

    std::cout << "指令: 低力闭合以夹取物体 (位置=255, 速度=100, 力=50)..." << std::endl;
    if (!gripper.move(255, 100, 50)) {
        std::cerr << "错误: 发送夹取指令失败。" << std::endl;
    } else {
        gripper.waitMotionComplete();
        std::cout << "移动完成。当前状态:" << std::endl;
        gripper.getStatus(status);
        status.print();
        std::cout << "请检查状态: 'Object Status' 是否为 'OUTER_GRIP_DETECTED' 并且 'Position' 小于 255？" << std::endl;
    }

    pressEnterToContinue();

    // --- 5. Gripper 禁用测试 ---
    std::cout << "--- [5] 测试 Gripper 禁用 ---" << std::endl;
    std::cout << "a) 禁用夹爪..." << std::endl;
    if (!gripper.disable()) {
        std::cerr << "错误: 禁用夹爪失败。" << std::endl;
    } else {
        std::cout << "成功: 禁用指令已发送。\n" << std::endl;
    }

    std::cout << "b) 获取禁用后的状态..." << std::endl;
    if (gripper.getStatus(status)) {
        status.print();
        std::cout << "请检查状态: 'Enabled' 是否为 'No'？" << std::endl;
    } else {
        std::cerr << "错误: 获取状态失败。\n" << std::endl;
    }

    // --- 6. 清理 ---
    std::cout << "\n--- [6] 清理并退出 ---" << std::endl;
    std::cout << "断开串口连接..." << std::endl;
    bus.disconnect();

    std::cout << "\n========= 测试完成 =========\n" << std::endl;

    return 0;
}