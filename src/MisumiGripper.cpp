// MisumiGripper.cpp
#include "MisumiGripper.hpp"
#include <modbus/modbus.h>
#include <iostream>
#include <vector>
#include <cmath> // for round

MisumiGripper::MisumiGripper(MisumiGripperBus& bus, int slave_id)
    : m_bus(bus), m_slave_id(slave_id) {}
MisumiGripper::~MisumiGripper() {}
bool MisumiGripper::isConnected() const {
    return m_is_connected;
}

std::string MisumiGripper::getLastError() const {
    return m_last_error.empty() ? m_bus.getLastError() : m_last_error;
}

bool MisumiGripper::enable() {
    // 根据文档6.2.1, 0x01: 使能执行开合动作搜索行程
    return m_bus.writeRegister(m_slave_id, GripperRegisters::INIT, 0x0001);
}

bool MisumiGripper::disable() {
    // 根据文档6.2.1, 0x00: 去使能
    return m_bus.writeRegister(m_slave_id, GripperRegisters::INIT, 0x0000);
}

bool MisumiGripper::moveTo(double position_mm, int speed_percent, int torque_percent) {
    if (!m_is_connected) {
        m_last_error = "Not connected.";
        return false;
    }
    
    // 数据转换和范围检查
    // 位置单位是 0.01mm
    uint16_t pos_val = static_cast<uint16_t>(std::round(position_mm * 100.0));
    // 速度和力矩是 1-100
    uint16_t speed_val = static_cast<uint16_t>(std::max(1, std::min(100, speed_percent)));
    uint16_t torque_val = static_cast<uint16_t>(std::max(1, std::min(100, torque_percent)));
    
    // 写入目标参数 (位置、速度、力矩)
    std::vector<uint16_t> values = {pos_val, speed_val, torque_val};
    if (!m_bus.writeRegisters(m_slave_id,GripperRegisters::TARGET_POS, values)) {
        return false;
    }
    
    // 触发运动
    // 根据文档6.2.6, 写入 1 到 0x0FA5 触发运动
    return m_bus.writeRegister(m_slave_id, GripperRegisters::TRIGGER_ACTION, 0x0001);
}

bool MisumiGripper::grip() {
    // 根据文档6.2.2, 0x05: 全力全速关闭
    return mbus_.writeRegister(m_slave_id,GripperRegisters::CONTROL_MODE, 0x0005);
}

bool MisumiGripper::open() {
    // 根据文档6.2.2, 0x04: 全力全速打开
    return mbus_.writeRegister(m_slave_id,GripperRegisters::CONTROL_MODE, 0x0004);
}

bool MisumiGripper::readStatus(GripperStatus& status) {
    const int NUM_REGS_TO_READ = 6;
    uint16_t buffer[NUM_REGS_TO_READ];
    
    if (!mbus_.readRegisters(m_slave_id, GripperRegisters::INIT_STATUS, NUM_REGS_TO_READ, buffer)) {
        return false;
    }
    
    // 解析数据
    status.is_enabled = (buffer[0] == 0x0003); // 0x1194 - 0x1194 = 0
    status.fault_code = buffer[1];             // 0x1195 - 0x1194 = 1
    status.grip_state = buffer[2];             // 0x1196 - 0x1194 = 2
    status.position_mm = buffer[3] / 100.0;    // 0x1197 - 0x1194 = 3
    status.speed_percent = buffer[4];          // 0x1198 - 0x1194 = 4
    status.torque_percent = buffer[5];         // 0x1199 - 0x1194 = 5
    
    return true;
}


bool MisumiGripper::stop() {
    // 根据文档 6.2.10, 写入 0x01 到 0x0FBE 立即停止运行
    return mbus_.writeRegister(m_slave_id, GripperRegisters::STOP_CONTROL, 0x0001);
}

bool MisumiGripper::setPreset(int preset_number, double position_mm, int speed_percent, int torque_percent) {
    if (preset_number < 1 || preset_number > 8) {
        m_last_error = "Preset number must be between 1 and 8.";
        return false;
    }

    // 数据转换
    uint16_t pos_val = static_cast<uint16_t>(std::round(position_mm * 100.0));
    uint16_t speed_val = static_cast<uint16_t>(std::max(1, std::min(100, speed_percent)));
    uint16_t torque_val = static_cast<uint16_t>(std::max(1, std::min(100, torque_percent)));

    // 计算预设参数的起始地址
    // 每个预设点占用 3 个寄存器 (位置, 速度, 力矩)
    int start_addr = GripperRegisters::PRESET_POS_1 + (preset_number - 1) * 3;

    // 使用 FC16 (Write Multiple Registers) 一次性写入三个参数
    std::vector<uint16_t> values = {pos_val, speed_val, torque_val};
    return mbus_.writeRegisters(m_slave_id, start_addr, values);
}

bool MisumiGripper::executePreset(int preset_number) {
    if (preset_number < 1 || preset_number > 8) {
        m_last_error = "Preset number must be between 1 and 8.";
        return false;
    }
    
    // 根据文档 6.2.2, 指令码从 0x08 (对应点1) 开始
    uint16_t command_code = 0x0007 + preset_number; 
    
    // 使用 FC06 (Write Single Register) 写入指令码到控制模式寄存器
    return mbus_.writeRegister(m_slave_id, GripperRegisters::CONTROL_MODE, command_code);
}

// Private helper methods
