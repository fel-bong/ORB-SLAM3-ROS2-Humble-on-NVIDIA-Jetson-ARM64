#!/usr/bin/env python3
"""
Live camera driver for ORB-SLAM3 monocular mode.
Bridges /camera/image/raw to ORB-SLAM3's expected topics.

Usage:
    ros2 run ros2_orb_slam3 live_mono_driver.py --ros-args -p settings_name:=IMX219
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float64
from cv_bridge import CvBridge


class LiveMonoDriver(Node):
    def __init__(self):
        super().__init__('live_mono_driver')
        
        # Parameters
        self.declare_parameter('settings_name', 'IMX219')
        self.declare_parameter('camera_topic', '/camera/image/raw')
        
        self.settings_name = self.get_parameter('settings_name').value
        self.camera_topic = self.get_parameter('camera_topic').value
        
        self.get_logger().info(f'Settings: {self.settings_name}')
        self.get_logger().info(f'Camera topic: {self.camera_topic}')
        
        self.br = CvBridge()
        self.send_config = True
        self.frame_count = 0
        
        # Publishers for ORB-SLAM3
        self.pub_config = self.create_publisher(
            String, 
            '/mono_py_driver/experiment_settings', 
            1
        )
        self.pub_img = self.create_publisher(
            Image, 
            '/mono_py_driver/img_msg', 
            1
        )
        self.pub_timestep = self.create_publisher(
            Float64, 
            '/mono_py_driver/timestep_msg', 
            1
        )
        
        # Subscriber for ACK
        self.sub_ack = self.create_subscription(
            String,
            '/mono_py_driver/exp_settings_ack',
            self.ack_callback,
            10
        )
        
        # Camera subscriber (will be created after handshake)
        self.sub_camera = None
        
        # QoS for camera (match your publisher)
        self.camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Timer for handshake
        self.handshake_timer = self.create_timer(0.1, self.handshake_callback)
        
        self.get_logger().info('LiveMonoDriver initialized, attempting handshake...')
    
    def ack_callback(self, msg):
        if msg.data == 'ACK' and self.send_config:
            self.get_logger().info('Handshake complete!')
            self.send_config = False
            self.handshake_timer.cancel()
            
            # Now subscribe to camera
            self.sub_camera = self.create_subscription(
                Image,
                self.camera_topic,
                self.camera_callback,
                self.camera_qos
            )
            self.get_logger().info(f'Subscribed to {self.camera_topic}')
    
    def handshake_callback(self):
        if self.send_config:
            msg = String()
            msg.data = self.settings_name
            self.pub_config.publish(msg)
    
    def camera_callback(self, msg):
        # Get timestamp in nanoseconds, convert to seconds as float
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # Publish timestep first (as the original driver does)
        timestep_msg = Float64()
        timestep_msg.data = timestamp
        self.pub_timestep.publish(timestep_msg)
        
        # Publish image
        self.pub_img.publish(msg)
        
        self.frame_count += 1
        if self.frame_count % 100 == 0:
            self.get_logger().info(f'Published {self.frame_count} frames')


def main(args=None):
    rclpy.init(args=args)
    node = LiveMonoDriver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()