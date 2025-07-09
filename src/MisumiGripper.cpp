// MisumiGripper.cpp
#include "MisumiGripper.hpp"
#include <modbus/modbus.h>
#include <iostream>
#include <vector>
#include <cmath> // for round

MisumiGripper::MisumiGripper(const std::string& device, int slave_id, int baud_rate, char parity, int data_bit, int stop_bit)
    : m_device(device), m_slave_id(slave_id), m_baud_rate(baud_rate), 
      m_parity(parity), m_data_bit(data_bit), m_stop_bit(stop_bit) {
    // 创建 Modbus RTU 上下文
    m_ctx = modbus_new_rtu(m_device.c_str(), m_baud_rate, m_parity, m_data_bit, m_stop_bit);
    if (m_ctx == nullptr) {
        m_last_error = "Failed to create Modbus RTU context.";
        std::cerr << m_last_error << std::endl;
    }
}

MisumiGripper::~MisumiGripper() {
    disconnect();
    if (m_ctx != nullptr) {
        modbus_free(m_ctx);
        m_ctx = nullptr;
    }
}

bool MisumiGripper::connect() {
    if (m_ctx == nullptr) {
        return false;
    }
    
    // 设置从站ID
    if (modbus_set_slave(m_ctx, m_slave_id) == -1) {
        m_last_error = "Failed to set slave ID: " + std::string(modbus_strerror(errno));
        return false;
    }

    // 建立连接
    if (modbus_connect(m_ctx) == -1) {
        m_last_error = "Connection failed: " + std::string(modbus_strerror(errno));
        return false;
    }
    
    // 设置响应超时时间
    modbus_set_response_timeout(m_ctx, 1, 0); // 1秒超时

    m_is_connected = true;
    m_last_error = "";
    return true;
}

void MisumiGripper::disconnect() {
    if (m_is_connected) {
        modbus_close(m_ctx);
        m_is_connected = false;
    }
}

bool MisumiGripper::isConnected() const {
    return m_is_connected;
}

std::string MisumiGripper::getLastError() const {
    return m_last_error;
}

bool MisumiGripper::enable() {
    // 根据文档6.2.1, 0x01: 使能执行开合动作搜索行程
    return writeRegister(GripperRegisters::INIT, 0x0001);
}

bool MisumiGripper::disable() {
    // 根据文档6.2.1, 0x00: 去使能
    return writeRegister(GripperRegisters::INIT, 0x0000);
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
    if (!writeRegisters(GripperRegisters::TARGET_POS, values)) {
        return false;
    }
    
    // 触发运动
    // 根据文档6.2.6, 写入 1 到 0x0FA5 触发运动
    return writeRegister(GripperRegisters::TRIGGER_ACTION, 0x0001);
}

bool MisumiGripper::grip() {
    // 根据文档6.2.2, 0x05: 全力全速关闭
    return writeRegister(GripperRegisters::CONTROL_MODE, 0x0005);
}

bool MisumiGripper::open() {
    // 根据文档6.2.2, 0x04: 全力全速打开
    return writeRegister(GripperRegisters::CONTROL_MODE, 0x0004);
}

bool MisumiGripper::readStatus(GripperStatus& status) {
    const int NUM_REGS_TO_READ = 6;
    uint16_t buffer[NUM_REGS_TO_READ];
    
    if (!readRegisters(GripperRegisters::INIT_STATUS, NUM_REGS_TO_READ, buffer)) {
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


// Private helper methods
bool MisumiGripper::writeRegister(int addr, uint16_t value) {
    if (!m_is_connected) {
        m_last_error = "Not connected.";
        return false;
    }
    if (modbus_write_register(m_ctx, addr, value) == -1) {
        m_last_error = "Failed to write register: " + std::string(modbus_strerror(errno));
        return false;
    }
    m_last_error = "";
    return true;
}

bool MisumiGripper::writeRegisters(int start_addr, const std::vector<uint16_t>& values) {
    if (!m_is_connected) {
        m_last_error = "Not connected.";
        return false;
    }
    if (modbus_write_registers(m_ctx, start_addr, values.size(), values.data()) == -1) {
        m_last_error = "Failed to write registers: " + std::string(modbus_strerror(errno));
        return false;
    }
    m_last_error = "";
    return true;
}

bool MisumiGripper::readRegisters(int start_addr, int num, uint16_t* dest) {
    if (!m_is_connected) {
        m_last_error = "Not connected.";
        return false;
    }
    // 注意: 状态寄存器是 Input Registers (功能码 0x04)
    if (modbus_read_input_registers(m_ctx, start_addr, num, dest) == -1) {
        m_last_error = "Failed to read input registers: " + std::string(modbus_strerror(errno));
        return false;
    }
    m_last_error = "";
    return true;
}