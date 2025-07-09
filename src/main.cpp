#include "MisumiGripper.h"
#include <iostream>
#include <thread> // for std::this_thread::sleep_for
#include <chrono> // for std::chrono::seconds

void printStatus(const GripperStatus& status) {
    std::cout << "------ Gripper Status ------" << std::endl;
    std::cout << "Enabled: " << (status.is_enabled ? "Yes" : "No") << std::endl;
    std::cout << "Fault Code: 0x" << std::hex << status.fault_code << std::dec << std::endl;
    std::cout << "Grip State: " << status.grip_state << std::endl;
    std::cout << "Position: " << status.position_mm << " mm" << std::endl;
    std::cout << "Speed: " << status.speed_percent << " %" << std::endl;
    std::cout << "Torque: " << status.torque_percent << " %" << std::endl;
    std::cout << "----------------------------" << std::endl;
}


int main() {
    // 替换为您的串口设备和夹爪ID
    const char* device = "/dev/ttyUSB0"; // Linux
    // const char* device = "COM3";      // Windows
    int slave_id = 9; // 默认出厂ID

    // 1. 创建夹爪对象
    MisumiGripper gripper(device, slave_id);

    // 2. 连接
    std::cout << "Connecting to gripper on " << device << "..." << std::endl;
    if (!gripper.connect()) {
        std::cerr << "Error: " << gripper.getLastError() << std::endl;
        return -1;
    }
    std::cout << "Connection successful." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 3. 使能夹爪
    std::cout << "Enabling gripper..." << std::endl;
    if (!gripper.enable()) {
        std::cerr << "Error enabling: " << gripper.getLastError() << std::endl;
        return -1;
    }
    // 等待使能完成 (搜索行程)
    std::cout << "Waiting for gripper to finish homing..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(5)); 

    // 4. 读取并打印状态
    GripperStatus status;
    if (gripper.readStatus(status)) {
        printStatus(status);
    } else {
        std::cerr << "Error reading status: " << gripper.getLastError() << std::endl;
    }

    // 5. 完全打开夹爪
    std::cout << "\nOpening gripper..." << std::endl;
    if (!gripper.open()) {
        std::cerr << "Error opening: " << gripper.getLastError() << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::seconds(3));
    if (gripper.readStatus(status)) {
        printStatus(status);
    }
    
    // 6. 移动到指定位置
    std::cout << "\nMoving to 10.0 mm with 50% speed and 80% torque..." << std::endl;
    if (!gripper.moveTo(10.0, 50, 80)) {
         std::cerr << "Error moving to position: " << gripper.getLastError() << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::seconds(3));
    if (gripper.readStatus(status)) {
        printStatus(status);
    }


    // 7. 完全关闭夹爪
    std::cout << "\nClosing gripper..." << std::endl;
    if (!gripper.grip()) {
        std::cerr << "Error closing: " << gripper.getLastError() << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::seconds(3));
    if (gripper.readStatus(status)) {
        printStatus(status);
    }
    
    // 8. 去使能
    std::cout << "\nDisabling gripper..." << std::endl;
    gripper.disable();
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 9. 断开连接 (析构函数会自动调用, 这里显式调用作为演示)
    std::cout << "Disconnecting..." << std::endl;
    gripper.disconnect();
    
    std::cout << "Demo finished." << std::endl;

    return 0;
}