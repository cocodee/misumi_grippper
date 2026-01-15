#include <pybind11/pybind11.h>
#include <pybind11/stl.h> // 用于 std::string 和 std::vector 的转换
#include "MisumiGripper.hpp" // 包含你的头文件
#include "MisumiGripperBus.hpp"
namespace py = pybind11;

PYBIND11_MODULE(misumi_gripper_py, m) {
    m.doc() = "Misumi Gripper Python Bindings";

    // 1. 绑定 GripperStatus 结构体
    py::class_<GripperStatus>(m, "MisumiGripperStatus")
        .def(py::init<>())
        .def_readwrite("is_enabled", &GripperStatus::is_enabled)
        .def_readwrite("fault_code", &GripperStatus::fault_code)
        .def_readwrite("grip_state", &GripperStatus::grip_state)
        .def_readwrite("position_mm", &GripperStatus::position_mm)
        .def_readwrite("speed_percent", &GripperStatus::speed_percent)
        .def_readwrite("torque_percent", &GripperStatus::torque_percent)
        .def("__repr__", [](const GripperStatus &s) {
            return "<GripperStatus enabled=" + std::to_string(s.is_enabled) +
                   ", state=" + std::to_string(s.grip_state) +
                   ", pos=" + std::to_string(s.position_mm) + ">";
        });

    // 2. 绑定 MisumiGripperBus 类
    py::class_<MisumiGripperBus>(m, "MisumiGripperBus")
        .def(py::init<const std::string &, int, char, int, int>(),
             py::arg("device"), 
             py::arg("baud_rate"), 
             py::arg("parity") = 'N', 
             py::arg("data_bit") = 8, 
             py::arg("stop_bit") = 1)
        .def("connect", &MisumiGripperBus::connect)
        .def("disconnect", &MisumiGripperBus::disconnect)
        .def("isConnected", &MisumiGripperBus::isConnected)
        .def("getLastError", &MisumiGripperBus::getLastError);

    // 3. 绑定 MisumiGripper 类
    py::class_<MisumiGripper>(m, "MisumiGripper")
        // 构造函数
        // keep_alive<1, 2>: 保证 Bus (参数2) 在 Gripper (参数1/this) 存活期间不被销毁
        .def(py::init<MisumiGripperBus&, int>(), 
             py::arg("bus"), 
             py::arg("slave_id"),
             py::keep_alive<1, 2>()) 
        
        .def("getLastError", &MisumiGripper::getLastError)
        .def("enable", &MisumiGripper::enable)
        .def("disable", &MisumiGripper::disable)
        
        .def("moveTo", &MisumiGripper::moveTo, 
             py::arg("position_mm"), py::arg("speed_percent"), py::arg("torque_percent"))
        
        .def("grip", &MisumiGripper::grip)
        .def("open", &MisumiGripper::open)
        
        // 原始 readStatus 需要传入一个 Status 对象
        .def("readStatus", &MisumiGripper::readStatus, py::arg("status"))
        
        // 添加一个 Pythonic 的辅助方法：直接返回 Status 对象或 None
        .def("get_status", [](MisumiGripper &self) -> py::object {
            GripperStatus status;
            if (self.readStatus(status)) {
                return py::cast(status);
            } else {
                return py::none();
            }
        }, "Helper function to get status directly as an object")

        .def("stop", &MisumiGripper::stop)
        
        .def("setPreset", &MisumiGripper::setPreset,
             py::arg("preset_number"), py::arg("position_mm"), py::arg("speed_percent"), py::arg("torque_percent"))
        
        .def("executePreset", &MisumiGripper::executePreset, py::arg("preset_number"));
}