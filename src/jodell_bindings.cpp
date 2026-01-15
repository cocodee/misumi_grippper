// bindings.cpp
#include <pybind11/pybind11.h>
#include <pybind11/stl.h> // 用于 std::string 等
#include <stdexcept>    // 用于抛出异常

// 包含您要绑定的头文件
#include "JodellGripper.hpp"
#include "GripperBus.hpp" // 假设 GripperBus.hpp 实际文件名为 gripper_bus.h

namespace py = pybind11;

// 使用 using 声明 py::arg 来简化代码
using py::arg;

PYBIND11_MODULE(jodell_gripper_py, m) {
    m.doc() = "Python bindings for the Jodell Gripper library";

    // --- 绑定枚举 ---
    py::enum_<ObjectDetectionStatus>(m, "ObjectDetectionStatus")
        .value("MOVING", ObjectDetectionStatus::MOVING, "手指正向指定位置移动")
        .value("INNER_GRIP_DETECTED", ObjectDetectionStatus::INNER_GRIP_DETECTED, "内撑模式接触到物体")
        .value("OUTER_GRIP_DETECTED", ObjectDetectionStatus::OUTER_GRIP_DETECTED, "外夹模式接触到物体")
        .value("NO_OBJECT_DETECTED", ObjectDetectionStatus::NO_OBJECT_DETECTED, "到达指定位置，未检测到物体")
        .value("UNKNOWN", ObjectDetectionStatus::UNKNOWN, "未知状态")
        .export_values(); // 允许在模块顶层访问枚举成员，如 m.MOVING

    py::enum_<ActivationStatus>(m, "ActivationStatus")
        .value("RESETTING", ActivationStatus::RESETTING, "处于复位或巡检状态")
        .value("ACTIVATING", ActivationStatus::ACTIVATING, "正在激活")
        .value("RESERVED", ActivationStatus::RESERVED, "未使用")
        .value("ACTIVATION_COMPLETE", ActivationStatus::ACTIVATION_COMPLETE, "激活完成")
        .export_values();

    // --- 绑定 GripperStatus 结构体 ---
    // 将其绑定为一个 Python 类
    py::class_<GripperStatus>(m, "JodellGripperStatus", py::module_local()) 
        .def(py::init<>()) // 默认构造函数
        .def_readwrite("enabled", &GripperStatus::enabled)
        .def_readwrite("is_moving", &GripperStatus::is_moving)
        .def_readwrite("activation_status", &GripperStatus::activation_status)
        .def_readwrite("object_status", &GripperStatus::object_status)
        .def_readwrite("position", &GripperStatus::position)
        .def_readwrite("speed", &GripperStatus::speed)
        .def_readwrite("force_current", &GripperStatus::force_current)
        .def_readwrite("bus_voltage", &GripperStatus::bus_voltage)
        .def_readwrite("temperature", &GripperStatus::temperature)
        .def("print", &GripperStatus::print, "打印状态到控制台 (C++ stdout)")
        // 添加一个 __repr__ 方法，使其在 Python 中打印更友好
        .def("__repr__", [](const GripperStatus &s) {
            return "<GripperStatus: enabled=" + std::to_string(s.enabled) +
                   ", moving=" + std::to_string(s.is_moving) +
                   ", pos=" + std::to_string(s.position) +
                   ", speed=" + std::to_string(s.speed) +
                   ", force=" + std::to_string(s.force_current) + ">";
        });

    // --- 绑定 GripperBus 类 ---
    // 注意：GripperBus 是不可拷贝的，pybind11 默认会处理好这一点
    py::class_<GripperBus>(m, "GripperBus")
        .def(py::init<const std::string&, int, char, int, int>(),
             arg("device"),
             arg("baud") = 115200,
             arg("parity") = 'N',
             arg("data_bit") = 8,
             arg("stop_bit") = 1)
        .def("connect", &GripperBus::connect, "连接到 Modbus 总线")
        .def("disconnect", &GripperBus::disconnect, "断开 Modbus 总线连接")
        .def("is_connected", &GripperBus::isConnected, "检查总线是否已连接");
        // 注意：getModbusContext() 返回一个裸指针，通常不建议暴露给 Python，
        // 除非有特殊需求。这里我们遵循最小暴露原则，不绑定它。

    // --- 绑定 JodellGripper 类 ---
    py::class_<JodellGripper>(m, "JodellGripper")
        .def(py::init<GripperBus&, int>(),
             arg("bus"),
             arg("slave_id"))
        .def("enable", &JodellGripper::enable, "使能夹爪 (激活)")
        .def("disable", &JodellGripper::disable, "禁用夹爪")
        .def("move", &JodellGripper::move,
             "控制夹爪移动到指定位置",
             arg("pos"), arg("speed"), arg("force"))
        
        // C++ 中的 getStatus(GripperStatus&) 是通过引用参数返回值的。
        // 这在 Python 中不常见。我们将其包装成一个直接返回 GripperStatus 对象的函数，
        // 如果 C++ 函数返回 false，则在 Python 中抛出异常，这更符合 Python 的风格。
        .def("get_status", [](JodellGripper &self) {
            GripperStatus status;
            if (self.getStatus(status)) {
                return status;
            }
            throw std::runtime_error("Failed to read gripper status.");
        }, "获取夹爪的当前全部状态")

        .def("wait_motion_complete", &JodellGripper::waitMotionComplete,
             "等待夹爪运动停止",
             arg("timeout_ms") = 5000);
}