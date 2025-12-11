import misumi_gripper_py as mg
import time

def main():
    # 1. 创建总线连接
    # 参数: device, baud, parity, data_bit, stop_bit
    bus = mg.MisumiGripperBus("/dev/ttyTHS1", 115200, 'N', 8, 1)
    
    if not bus.connect():
        print(f"连接失败: {bus.getLastError()}")
        return

    # 2. 创建夹爪对象 (Slave ID = 1)
    gripper = mg.MisumiGripper(bus, 27)

    # 3. 使能
    print("正在使能夹爪...")
    if gripper.enable():
        print("使能指令发送成功")
    else:
        print(f"使能失败: {gripper.getLastError()}")

    # 等待一会
    time.sleep(1)

    # 4. 读取状态 (方式 A: Pythonic 方式)
    status = gripper.get_status()
    if status:
        print(f"当前位置: {status.position_mm}mm, 状态码: {status.grip_state}")
    else:
        print("读取状态失败")

    # 5. 读取状态 (方式 B: C++ 风格)
    raw_status = mg.GripperStatus()
    if gripper.readStatus(raw_status):
        print(f"详细状态: {raw_status}") # 会调用绑定的 __repr__
    
    # 6. 移动
    gripper.moveTo(0.0, 100, 50) # 位置10mm, 速度50%, 力矩30%
    
    time.sleep(2)
    gripper.moveTo(20.0, 50, 50)
    time.sleep(2)
    gripper.moveTo(0.0, 50, 50)
    time.sleep(5)
    # 7. 关闭连接
    bus.disconnect()

if __name__ == "__main__":
    main()