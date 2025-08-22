// src/main.cpp
// 注意：请确保头文件名与您的项目文件一致
#include "GripperBus.hpp"   // 或者 "GripperBus.hpp"
#include "JodellGripper.hpp"       // 或者 "JodellGrippper.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <limits> // 用于 std::numeric_limits
#include <string> // 用于 std::stoi

// --- 辅助工具 ---

/**
 * @class ScopedTimer
 * @brief 一个利用RAII模式的简单计时器。
 *        在创建时记录开始时间，在销毁时（离开作用域时）打印经过的时间。
 */
class ScopedTimer {
public:
    ScopedTimer(const std::string& name)
        : name_(name), start_time_(std::chrono::steady_clock::now()) {}

    ~ScopedTimer() {
        auto end_time = std::chrono::steady_clock::now();
        auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time_).count();
        std::cout << "\n>>> \"" << name_ << "\" completed in " << duration_ms << " ms. <<<\n" << std::endl;
    }

private:
    std::string name_;
    std::chrono::time_point<std::chrono::steady_clock> start_time_;
};

// 打印程序用法
void printUsage(const char* prog_name) {
    std::cerr << "\nUsage: " << prog_name << " <serial_port> <slave_id>\n";
    std::cerr << "  <serial_port>: The serial device name (e.g., /dev/ttyUSB0 on Linux, COM3 on Windows)\n";
    std::cerr << "  <slave_id>: The Modbus slave ID of the gripper (e.g., 9)\n\n";
    std::cerr << "Example (Linux):   " << prog_name << " /dev/ttyUSB0 9\n";
    std::cerr << "Example (Windows): " << prog_name << " COM3 9\n";
}

// 辅助函数：等待用户按 Enter 键继续
void pressEnterToContinue() {
    std::cout << "... 按 Enter 键继续 ...";
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cin.get();
    std::cout << std::endl;
}

// --- 主函数 ---

int main(int argc, char* argv[]) {
    // --- 0. 参数解析与验证 ---
    if (argc != 3) {
        printUsage(argv[0]);
        return 1;
    }
    std::string SERIAL_PORT = argv[1];
    int SLAVE_ID = 0;
    try {
        SLAVE_ID = std::stoi(argv[2]);
    } catch (const std::exception& e) {
        std::cerr << "Error: Invalid Slave ID '" << argv[2] << "'. Must be a number." << std::endl;
        printUsage(argv[0]);
        return 1;
    }
    if (SLAVE_ID < 1 || SLAVE_ID > 247) {
        std::cerr << "Error: Slave ID must be between 1 and 247." << std::endl;
        return 1;
    }

    std::cout << "========= Jodell EPG Gripper Library Test Suite =========\n";
    std::cout << "Config: Port=" << SERIAL_PORT << ", Slave ID=" << SLAVE_ID << "\n" << std::endl;

    GripperBus bus(SERIAL_PORT);
    JodellGripper gripper(bus, SLAVE_ID);
    GripperStatus status;

    // --- 1. GripperBus 连接测试 ---
    std::cout << "--- [1] 测试 GripperBus 连接 ---" << std::endl;
    {
        ScopedTimer timer("Test [1] GripperBus Connection");
        std::cout << "a) 尝试连接一个无效的串口 '/dev/nonexistent' (预期失败)..." << std::endl;
        GripperBus invalid_bus("/dev/nonexistent");
        if (!invalid_bus.connect()) {
            std::cout << "成功: 无效串口连接失败，符合预期。\n" << std::endl;
        } else {
            std::cerr << "失败: 竟然成功连接到了一个无效的串口！\n" << std::endl;
            invalid_bus.disconnect();
        }

        std::cout << "b) 尝试连接到配置的串口: " << SERIAL_PORT << " ..." << std::endl;
        if (!bus.connect()) {
            std::cerr << "错误: 无法连接到串口 " << SERIAL_PORT << ". 请检查端口名、权限和硬件连接。" << std::endl;
            return -1;
        }
        std::cout << "成功: 串口连接成功。\n";
    }

    // --- 2. Gripper 初始化和使能测试 ---
    std::cout << "--- [2] 测试 Gripper 初始化和使能 ---" << std::endl;
    {
        ScopedTimer timer("Test [2] Gripper Init & Enable");
        std::cout << "a) 使能 (激活) 夹爪 ID: " << SLAVE_ID << "..." << std::endl;
        if (!gripper.enable()) {
            std::cerr << "错误: 使能夹爪失败。请检查夹爪是否已上电且连接正常。" << std::endl;
            bus.disconnect();
            return -1;
        }
        std::cout << "夹爪正在进行初始化动作，请稍候..." << std::endl;
        if (!gripper.waitMotionComplete(10000)) {
            std::cerr << "警告: 等待初始化完成超时。" << std::endl;
        } else {
            std::cout << "成功: 夹爪初始化完成。\n" << std::endl;
        }
        std::cout << "b) 获取夹爪初始化后的状态..." << std::endl;
        if (gripper.getStatus(status)) {
            status.print();
        } else {
            std::cerr << "错误: 获取状态失败。\n";
        }
    }
    pressEnterToContinue();

    // --- 3. Gripper 移动功能测试 ---
    std::cout << "--- [3] 测试 Gripper 移动功能 (move) ---" << std::endl;
    {
        ScopedTimer timer("Test [3] Gripper Move Functionality");
        std::cout << "a) 指令: 全速全力打开 (位置=0, 速度=255, 力=255)..." << std::endl;
        gripper.move(0, 255, 255);
        gripper.waitMotionComplete();
        gripper.getStatus(status);
        status.print();
        std::this_thread::sleep_for(std::chrono::seconds(1));

        std::cout << "\nb) 指令: 半速半力闭合 (位置=255, 速度=128, 力=128)..." << std::endl;
        gripper.move(255, 128, 128);
        gripper.waitMotionComplete();
        gripper.getStatus(status);
        status.print();
        std::this_thread::sleep_for(std::chrono::seconds(1));

        std::cout << "\nc) 指令: 低速移动到中间位置 (位置=150, 速度=80, 力=200)..." << std::endl;
        gripper.move(150, 80, 200);
        gripper.waitMotionComplete();
        gripper.getStatus(status);
        status.print();
    }
    pressEnterToContinue();
    
    // --- 4. 夹取物体测试 (手动) ---
    std::cout << "--- [4] 测试物体检测 (手动操作) ---" << std::endl;
    std::cout << "请在夹爪路径中放置一个物体。" << std::endl;
    pressEnterToContinue();
    {
        ScopedTimer timer("Test [4] Object Detection");
        std::cout << "指令: 低力闭合以夹取物体 (位置=255, 速度=100, 力=50)..." << std::endl;
        gripper.move(255, 100, 50);
        gripper.waitMotionComplete();
        std::cout << "移动完成。当前状态:" << std::endl;
        gripper.getStatus(status);
        status.print();
        std::cout << "请检查状态: 'Object Status' 是否为 'OUTER_GRIP_DETECTED' 并且 'Position' 小于 255？" << std::endl;
    }
    pressEnterToContinue();

    // --- 5. Gripper 禁用测试 ---
    std::cout << "--- [5] 测试 Gripper 禁用 ---" << std::endl;
    {
        ScopedTimer timer("Test [5] Gripper Disable");
        std::cout << "a) 禁用夹爪..." << std::endl;
        gripper.disable();
        std::cout << "成功: 禁用指令已发送。\n" << std::endl;
        std::cout << "b) 获取禁用后的状态..." << std::endl;
        if (gripper.getStatus(status)) {
            status.print();
            std::cout << "请检查状态: 'Enabled' 是否为 'No'？" << std::endl;
        } else {
            std::cerr << "错误: 获取状态失败。\n";
        }
    }

    // --- 6. 清理 ---
    std::cout << "--- [6] 清理并退出 ---" << std::endl;
    {
        ScopedTimer timer("Test [6] Cleanup");
        std::cout << "断开串口连接..." << std::endl;
        bus.disconnect();
    }

    std::cout << "\n========= 测试全部完成 =========\n" << std::endl;

    return 0;
}