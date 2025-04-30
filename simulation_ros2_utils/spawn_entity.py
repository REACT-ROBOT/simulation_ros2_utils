import os
import rclpy
from rclpy.node import Node
import subprocess
from ament_index_python.packages import get_package_share_directory
import shutil
import xml.etree.ElementTree as ET
from simulation_interfaces.srv import SpawnEntity
from geometry_msgs.msg import Pose
from builtin_interfaces.msg import Time
from rclpy.qos import QoSProfile
import numpy as np
import math

def euler_to_quaternion(roll: float, pitch: float, yaw: float):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw

class SpawnEntityNode(Node):
    def __init__(self):
        super().__init__('spawn_entity_node')

        self.declare_parameter('urdf_path', '')
        urdf_path = self.get_parameter('urdf_path').get_parameter_value().string_value
        if urdf_path == '':
            return
        
        self.declare_parameter('robot_name', 'robot')
        robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        
        self.declare_parameter('x', 0.0)
        robot_x = self.get_parameter('x').get_parameter_value().double_value
        self.declare_parameter('y', 0.0)
        robot_y = self.get_parameter('y').get_parameter_value().double_value
        self.declare_parameter('z', 0.0)
        robot_z = self.get_parameter('z').get_parameter_value().double_value
        self.declare_parameter('R', 0.0)
        robot_roll = self.get_parameter('R').get_parameter_value().double_value
        self.declare_parameter('P', 0.0)
        robot_pitch = self.get_parameter('P').get_parameter_value().double_value
        self.declare_parameter('Y', 0.0)
        robot_yaw = self.get_parameter('Y').get_parameter_value().double_value
        
        # オイラー角からクォータニオンへの変換
        qx, qy, qz, qw = euler_to_quaternion(robot_roll, robot_pitch, robot_yaw)

        # SpawnEntityサービスクライアントを作成
        self.spawn_client = self.create_client(SpawnEntity, 'spawn_entity')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('spawn_entity service not available, waiting...')
        
        self.get_logger().info("Reading URDF file...")
        
        # URDFファイルの内容を読み込む
        urdf_file = open(urdf_path, 'r')
        urdf_content = urdf_file.read()
        urdf_file.close()
        
        # SpawnEntityリクエストの作成
        request = SpawnEntity.Request()
        request.name = robot_name
        request.uri = 'file://' + urdf_path
        request.resource_string = urdf_content
        request.entity_namespace = ''
        request.allow_renaming = False
        
        # 初期ポーズを設定
        request.initial_pose.header.stamp = Time()
        request.initial_pose.header.frame_id = ''
        request.initial_pose.pose.position.x = robot_x
        request.initial_pose.pose.position.y = robot_y
        request.initial_pose.pose.position.z = robot_z
        request.initial_pose.pose.orientation.x = qx
        request.initial_pose.pose.orientation.y = qy
        request.initial_pose.pose.orientation.z = qz
        request.initial_pose.pose.orientation.w = qw
        
        self.get_logger().info("Sending spawn request...")
        
        # サービス呼び出し
        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.result.result == 0:  # SUCCESS
                self.get_logger().info(f"Entity '{request.name}' spawned successfully")
            else:
                self.get_logger().error(f"Failed to spawn entity: {response.result.error_message}")
        else:
            self.get_logger().error("Service call failed")

def main(args=None):
    rclpy.init(args=args)
    
    node = SpawnEntityNode()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

