#include "MisumiGripperBus.hpp"
#include <iostream>

MisumiGripperBus::MisumiGripperBus(const std::string& device, int baud_rate, char parity, int data_bit, int stop_bit)
    : m_device(device), m_baud_rate(baud_rate), 
      m_parity(parity), m_data_bit(data_bit), m_stop_bit(stop_bit) {
    m_ctx = modbus_new_rtu(m_device.c_str(), m_baud_rate, m_parity, m_data_bit, m_stop_bit);
    if (m_ctx == nullptr) {
        m_last_error = "Failed to create Modbus RTU context.";
        std::cerr << m_last_error << std::endl;
    }
    //std::cout << "[DEBUG] libmodbus debugging enabled." << std::endl;
    modbus_set_debug(m_ctx, TRUE);
    // 设置响应超时时间为 1 秒
    modbus_set_response_timeout(m_ctx, 1, 0);  
}

MisumiGripperBus::~MisumiGripperBus() {
    disconnect();
    if (m_ctx != nullptr) {
        modbus_free(m_ctx);
    }
}

bool MisumiGripperBus::connect() {
    if (m_ctx == nullptr) return false;
    if (modbus_connect(m_ctx) == -1) {
        m_last_error = "Connection failed: " + std::string(modbus_strerror(errno));
        return false;
    }
    modbus_set_response_timeout(m_ctx, 1, 0);
    m_is_connected = true;
    m_last_error = "";
    return true;
}

void MisumiGripperBus::disconnect() {
    if (m_is_connected) {
        modbus_close(m_ctx);
        m_is_connected = false;
    }
}

bool MisumiGripperBus::isConnected() const {
    return m_is_connected;
}

std::string MisumiGripperBus::getLastError() const {
    return m_last_error;
}

// --- Private helper methods now take slave_id ---

bool MisumiGripperBus::writeRegister(int slave_id, int addr, uint16_t value) {
    if (!m_is_connected) {
        m_last_error = "Not connected";
        return false;
    }
    
    // 设置从机 ID
    if (modbus_set_slave(m_ctx, slave_id) == -1) {
        m_last_error = "Invalid slave ID: " + std::string(modbus_strerror(errno));
        return false;
    }

    // 写入寄存器
    if (modbus_write_register(m_ctx, addr, value) == -1) {
        m_last_error = "Write failed: " + std::string(modbus_strerror(errno));
        return false;
    }
    return true;
}

bool MisumiGripperBus::writeRegisters(int slave_id, int start_addr, const std::vector<uint16_t>& values) {
    if (!m_is_connected) {
        m_last_error = "Not connected";
        return false;
    }
    
    if (modbus_set_slave(m_ctx, slave_id) == -1) {
        m_last_error = "Invalid slave ID: " + std::string(modbus_strerror(errno));
        return false;
    }

    if (modbus_write_registers(m_ctx, start_addr, values.size(), values.data()) == -1) {
        m_last_error = "Write registers failed: " + std::string(modbus_strerror(errno));
        return false;
    }
    return true;
}

bool MisumiGripperBus::readRegisters(int slave_id, int start_addr, int num, uint16_t* dest) {
    if (!m_is_connected) {
        m_last_error = "Not connected";
        return false;
    }
    
    if (modbus_set_slave(m_ctx, slave_id) == -1) {
        m_last_error = "Invalid slave ID: " + std::string(modbus_strerror(errno));
        return false;
    }

    // 【注意】这里如果你确定是功能码04则保持不变
    // 如果是功能码03 (Holding Registers)，请改为 modbus_read_registers
    if (modbus_read_input_registers(m_ctx, start_addr, num, dest) == -1) {
        m_last_error = "Read failed: " + std::string(modbus_strerror(errno));
        return false;
    }
    return true;
}