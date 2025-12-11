import misumi_gripper_py as mg
import time
import argparse
import sys

def run_gripper_control(device_name, client_id):
    """
    执行夹爪控制逻辑
    :param device_name: 串口设备路径 (例如 /dev/ttyTHS1)
    :param client_id: Modbus 从站 ID (例如 27)
    """
    print(f"--- 开始初始化 ---")
    print(f"目标设备: {device_name}")
    print(f"目标 ID : {client_id}")

    # 1. 创建总线连接
    # 参数: device, baud, parity, data_bit, stop_bit
    # 注意：波特率等参数保持默认，如果需要也可以提取为参数
    bus = mg.MisumiGripperBus(device_name, 115200, 'N', 8, 1)
    
    if not bus.connect():
        print(f"连接失败: {bus.getLastError()}")
        return

    # 2. 创建夹爪对象 (使用传入的 client_id)
    gripper = mg.MisumiGripper(bus, client_id)

    # 3. 使能
    print("正在使能夹爪...")
    if gripper.enable():
        print("使能指令发送成功")
    else:
        print(f"使能失败: {gripper.getLastError()}")
        bus.disconnect()
        return

    # 等待一会
    time.sleep(1)

    # 4. 读取状态 (方式 A: Pythonic 方式)
    status = gripper.get_status()
    if status:
        print(f"当前位置: {status.position_mm:.2f}mm, 状态码: {status.grip_state}")
    else:
        print("读取状态失败")

    # 5. 读取状态 (方式 B: C++ 风格)
    raw_status = mg.GripperStatus()
    if gripper.readStatus(raw_status):
        # 假设 C++ 绑定实现了 __repr__ 或 __str__
        print(f"详细状态: {raw_status}") 
    
    # 6. 移动测试
    print("开始移动测试...")
    
    # 动作 1: 移动到 0.0mm (张开/闭合视具体机械结构而定)
    target_pos = 0.0
    print(f"移动到 {target_pos}mm...")
    gripper.moveTo(target_pos, 100, 50) # 位置, 速度100%, 力矩50%
    time.sleep(5)
    
    # 动作 2: 移动到 20.0mm
    target_pos = 20.0
    print(f"移动到 {target_pos}mm...")
    gripper.moveTo(target_pos, 100, 50)
    time.sleep(5)
    
     # 动作 2: 移动到 20.0mm
    target_pos = 40.0
    print(f"移动到 {target_pos}mm...")
    gripper.moveTo(target_pos, 100, 50)
    time.sleep(5)
       
    # 动作 3: 回到 0.0mm
    #target_pos = 0.0
    #print(f"移动到 {target_pos}mm...")
    #gripper.moveTo(target_pos, 100, 50)
    #time.sleep(5) # 等待动作完成

    # 7. 关闭连接
    print("测试结束，断开连接")
    bus.disconnect()

if __name__ == "__main__":
    # 创建参数解析器
    parser = argparse.ArgumentParser(description="Misumi Gripper Control Test Script")
    
    # 添加 device 参数，默认值为 /dev/ttyTHS1
    parser.add_argument(
        '-d', '--device', 
        type=str, 
        default='/dev/ttyTHS1', 
        help='Serial port device path (default: /dev/ttyTHS1)'
    )
    
    # 添加 id 参数，默认值为 27
    parser.add_argument(
        '-i', '--id', 
        type=int, 
        default=27, 
        help='Gripper Client/Slave ID (default: 27)'
    )

    # 解析参数
    args = parser.parse_args()

    try:
        run_gripper_control(args.device, args.id)
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    except Exception as e:
        print(f"\n发生错误: {e}")