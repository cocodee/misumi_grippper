// src/gripper.cpp
#include "JodellGripper.hpp"
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>

// GripperStatus 结构体的打印函数
void GripperStatus::print() const {
    std::cout << "--- Gripper Status ---" << std::endl;
    std::cout << "Enabled: " << (enabled ? "Yes" : "No") << std::endl;
    std::cout << "Is Moving: " << (is_moving ? "Yes" : "No") << std::endl;
    
    std::cout << "Activation Status: ";
    switch (activation_status) {
        case ActivationStatus::RESETTING: std::cout << "Resetting" << std::endl; break;
        case ActivationStatus::ACTIVATING: std::cout << "Activating" << std::endl; break;
        case ActivationStatus::ACTIVATION_COMPLETE: std::cout << "Activation Complete" << std::endl; break;
        default: std::cout << "Unknown" << std::endl; break;
    }

    std::cout << "Object Status: ";
    switch (object_status) {
        case ObjectDetectionStatus::MOVING: std::cout << "Moving" << std::endl; break;
        case ObjectDetectionStatus::INNER_GRIP_DETECTED: std::cout << "Inner Grip Detected" << std::endl; break;
        case ObjectDetectionStatus::OUTER_GRIP_DETECTED: std::cout << "Outer Grip Detected" << std::endl; break;
        case ObjectDetectionStatus::NO_OBJECT_DETECTED: std::cout << "Position Reached, No Object" << std::endl; break;
        default: std::cout << "Unknown" << std::endl; break;
    }

    std::cout << "Position: " << static_cast<int>(position) << std::endl;
    std::cout << "Speed: " << static_cast<int>(speed) << std::endl;
    std::cout << "Force/Current: " << static_cast<int>(force_current) << std::endl;
    std::cout << "Bus Voltage: " << static_cast<int>(bus_voltage) << std::endl;
    std::cout << "Temperature: " << static_cast<int>(temperature) << std::endl;
    std::cout << "----------------------" << std::endl;
}


JodellGripper::JodellGripper(GripperBus& bus, int slave_id) : bus_(bus), slave_id_(slave_id) {}

bool JodellGripper::enable() {
    modbus_t* ctx = bus_.getModbusContext();
    if (!ctx || !bus_.isConnected()) return false;

    modbus_set_slave(ctx, slave_id_);
    
    // 写入 0x0001 到控制寄存器 (0x03E8) 来使能夹爪
    // rACT = 1 (bit 0)
    /*
    if (modbus_write_register(ctx, REG_CONTROL, 0x0001) == -1) {
        std::cerr << "Failed to enable gripper " << slave_id_ << ": " << modbus_strerror(errno) << std::endl;
        return false;
    }
    */
    uint16_t data_to_write[1] = {0x0001};

    // 2. 调用 modbus_write_registers
    //    参数分别为: 上下文, 起始地址, 写入寄存器数量 (1), 数据指针
    if (modbus_write_registers(ctx, REG_CONTROL, 1, data_to_write) == -1) {
        std::cerr << "Failed to enable gripper " << slave_id_ << " using 'write multiple': " << modbus_strerror(errno) << std::endl;
        return false;
    }
    return true;
}

bool JodellGripper::disable() {
    modbus_t* ctx = bus_.getModbusContext();
    if (!ctx || !bus_.isConnected()) return false;

    modbus_set_slave(ctx, slave_id_);

    // 写入 0x0000 到控制寄存器 (0x03E8) 来禁用夹爪
    // rACT = 0 (bit 0)
    if (modbus_write_register(ctx, REG_CONTROL, 0x0000) == -1) {
        std::cerr << "Failed to disable gripper " << slave_id_ << ": " << modbus_strerror(errno) << std::endl;
        return false;
    }
    return true;
}

bool JodellGripper::move(uint8_t pos, uint8_t speed, uint8_t force) {
    modbus_t* ctx = bus_.getModbusContext();
    if (!ctx || !bus_.isConnected()) return false;
    
    modbus_set_slave(ctx, slave_id_);

    // 根据文档 P12, 有参模式下需要将控制寄存器与位置、速度、力寄存器一起写入
    // 这需要使用功能码 16 (0x10) Write Multiple Registers
    std::vector<uint16_t> data(3);

    // 1. 设置控制寄存器 (0x03E8)
    // rACT=1 (bit 0), rMODE=0 (bit 1, 有参模式), rGTO=1 (bit 3)
    // Value = 0b00001001 = 0x09
    data[0] = 0x0009;

    // 2. 设置位置寄存器 (0x03E9)
    // 位置值在 高字节
    data[1] = static_cast<uint16_t>(pos) << 8;

    // 3. 设置速度/力寄存器 (0x03EA)
    // 速度值在 低字节, 力值在 高字节
    data[2] = (static_cast<uint16_t>(force) << 8) | speed;

    if (modbus_write_registers(ctx, REG_CONTROL, 3, data.data()) == -1) {
        std::cerr << "Failed to send move command to gripper " << slave_id_ << ": " << modbus_strerror(errno) << std::endl;
        return false;
    }
    return true;
}

bool JodellGripper::getStatus(GripperStatus& status) {
    modbus_t* ctx = bus_.getModbusContext();
    if (!ctx || !bus_.isConnected()) return false;
    
    modbus_set_slave(ctx, slave_id_);

    // 文档提到读取状态使用功能码 0x04 (Read Input Registers)
    // 一次性读取 4 个寄存器 (0x07D0 to 0x07D3)
    std::vector<uint16_t> regs(4);
    if (modbus_read_input_registers(ctx, REG_STATUS, 4, regs.data()) == -1) {
        std::cerr << "Failed to get status from gripper " << slave_id_ << ": " << modbus_strerror(errno) << std::endl;
        return false;
    }

    // 解析状态寄存器 0x07D0
    uint16_t status_reg = regs[0];
    status.enabled = (status_reg & 0b00000011) == 0x01;
    status.is_moving = (status_reg & 0b00001000) != 0;
    status.activation_status = static_cast<ActivationStatus>((status_reg >> 4) & 0x03);
    status.object_status = static_cast<ObjectDetectionStatus>((status_reg >> 6) & 0x03);

    // 解析位置状态寄存器 0x07D1
    // 位置在 高字节
    status.position = (regs[1] >> 8) & 0xFF;

    // 解析速度/力状态寄存器 0x07D2
    // 速度在 低字节, 力在 高字节
    status.speed = regs[2] & 0xFF;
    status.force_current = (regs[2] >> 8) & 0xFF;

    // 解析电压/温度状态寄存器 0x07D3
    // 电压在 低字节, 温度在 高字节
    status.bus_voltage = regs[3] & 0xFF;
    status.temperature = (regs[3] >> 8) & 0xFF;

    return true;
}


bool JodellGripper::waitMotionComplete(int timeout_ms) {
    auto start_time = std::chrono::steady_clock::now();
    while (true) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count();

        if (elapsed_time > timeout_ms) {
            std::cerr << "Timeout waiting for motion to complete." << std::endl;
            return false;
        }

        GripperStatus status;
        if (getStatus(status)) {
            // gGTO=0 表示运动停止, gSTA=3 表示激活完成
            if (!status.is_moving && status.activation_status == ActivationStatus::ACTIVATION_COMPLETE) {
                return true;
            }
        } else {
            // 如果读取状态失败，也退出等待
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}