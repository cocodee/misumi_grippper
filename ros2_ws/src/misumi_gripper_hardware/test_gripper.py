#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from rclpy.publisher import Publisher
import time
from sensor_msgs.msg import JointState
class GripperCommandPublisher(Node):
    """
    一个用于测试 ForwardCommandController 的 Python 节点。
    它通过发布 std_msgs/msg/Float64MultiArray 消息来控制夹爪。
    """
    def __init__(self):
        super().__init__('gripper_topic_test_publisher')
        
        # Topic 名称由控制器名称（misumi_gripper_controller）和固定的后缀（/commands）组成
        topic_name = '/misumi_gripper_controller/commands'
        
        # 创建一个发布者
        # 消息类型是 std_msgs.msg.Float64MultiArray
        self.publisher_ = self.create_publisher(Float64MultiArray, topic_name, 10)
        
        self.subscription = self.node.create_subscription(
            JointState,
            '/misumi_gripper/joint_states',  # 假设你已经将话题重命名
            self.joint_state_callback,
            10)
        # 等待一秒钟，确保发布者和订阅者之间建立连接
        time.sleep(1.0)
        
        self.get_logger().info(f"创建 Publisher，发布到 '{topic_name}'...")


    def joint_state_callback(self, msg):
        """回调函数，保存最新的 /joint_states 消息"""
        self.last_joint_state = msg
        print(f'gripper_joint_states:{msg}')

    def send_position_command(self, position: float):
        """
        发送一个位置指令到夹爪控制器。

        Args:
            position (float): 目标关节位置（单位：米）。
        """
        # 创建消息
        msg = Float64MultiArray()
        
        # ForwardCommandController 期望一个数组，即使只有一个关节。
        # 数组中的值对应于 YAML 文件中 `joints` 列表的顺序。
        msg.data = [position]
        
        # 发布消息
        self.publisher_.publish(msg)
        self.get_logger().info(f"已发布指令 -> 目标位置: {position:.4f} m")

def main(args=None):
    rclpy.init(args=args)

    # 创建测试节点实例
    test_publisher = GripperCommandPublisher()

    # --- 重要假设 ---
    # 由于我们直接发送物理单位（米），我们需要知道夹爪的实际行程。
    # 这里我们假设夹爪的行程是 0 到 40mm (0.0 到 0.04 米)。
    # 请根据您 URDF 文件中定义的实际关节限位来修改这些值！
    POS_OPEN = 0.04  # 假设完全张开是 40mm
    POS_CLOSED = 0.0  # 假设完全闭合是 0mm
    POS_HALF = 0.02   # 中间位置

    # --- 开始测试序列 ---
    try:
        # 1. 完全打开夹爪
        test_publisher.get_logger().info(f"\n--- 测试 1: 完全打开夹爪到 {POS_OPEN} m ---")
        test_publisher.send_position_command(POS_OPEN)
        time.sleep(2.0)  # 等待夹爪物理移动完成

        # 2. 完全闭合夹爪
        test_publisher.get_logger().info(f"\n--- 测试 2: 完全闭合夹爪到 {POS_CLOSED} m ---")
        test_publisher.send_position_command(POS_CLOSED)
        time.sleep(2.0)

        # 3. 移动到中间位置
        test_publisher.get_logger().info(f"\n--- 测试 3: 移动到中间位置 {POS_HALF} m ---")
        test_publisher.send_position_command(POS_HALF)
        time.sleep(2.0)

        # 4. 再次打开，确认可以重复操作
        test_publisher.get_logger().info(f"\n--- 测试 4: 再次打开夹爪到 {POS_OPEN} m ---")
        test_publisher.send_position_command(POS_OPEN)
        time.sleep(2.0)

    except KeyboardInterrupt:
        test_publisher.get_logger().info("测试被用户中断")
    finally:
        # 清理并关闭节点
        test_publisher.get_logger().info("所有测试完成。")
        test_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()