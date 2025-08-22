// src/gripper_bus.cpp
#include "GripperBus.hpp"
#include <iostream>

GripperBus::GripperBus(const std::string& device, int baud, char parity, int data_bit, int stop_bit)
    : ctx_(nullptr),
      device_(device),
      baud_(baud),
      parity_(parity),
      data_bit_(data_bit),
      stop_bit_(stop_bit),
      is_connected_(false) {
    ctx_ = modbus_new_rtu(device_.c_str(), baud_, parity_, data_bit_, stop_bit_);
    if (ctx_ == nullptr) {
        std::cerr << "Failed to create modbus context for " << device_ << std::endl;
    } else {
        //std::cout << "[DEBUG] libmodbus debugging enabled." << std::endl;
        //modbus_set_debug(ctx_, TRUE);
        // 设置响应超时时间为 1 秒
        modbus_set_response_timeout(ctx_, 1, 0);
    }
}

GripperBus::~GripperBus() {
    disconnect();
    if (ctx_) {
        modbus_free(ctx_);
    }
}

bool GripperBus::connect() {
    if (!ctx_) return false;
    if (modbus_connect(ctx_) == -1) {
        std::cerr << "Modbus connection failed for " << device_ << ": " << modbus_strerror(errno) << std::endl;
        is_connected_ = false;
        return false;
    }
    std::cout << "Modbus connection successful on " << device_ << std::endl;
    is_connected_ = true;
    return true;
}

void GripperBus::disconnect() {
    if (is_connected_ && ctx_) {
        modbus_close(ctx_);
        is_connected_ = false;
        std::cout << "Modbus connection closed on " << device_ << std::endl;
    }
}

bool GripperBus::isConnected() const {
    return is_connected_;
}

modbus_t* GripperBus::getModbusContext() {
    return ctx_;
}