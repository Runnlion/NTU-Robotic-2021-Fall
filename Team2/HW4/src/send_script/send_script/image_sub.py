#!/usr/bin/env python
from cv_bridge import CvBridge
from ImageLoader import Calibration
from Gripper import ArmControl, Gripper

from sensor_msgs.msg import Image
from tm_msgs.srv import *
from tm_msgs.msg import *
import rclpy

from rclpy.node import Node

import sys
sys.path.append(
    '/home/robot/colcon_ws/install/tm_msgs/lib/python3.6/site-packages')
sys.path.append('/home/robot/workspace2/team2_ws/src/send_script/send_script')


class ImageSub(Node):
    def __init__(self, nodeName):
        super().__init__(nodeName)
        self.subscription = self.create_subscription(Image,
                                                     'techman_image', self.image_callback, 10)
        self.subscription

    def image_callback(self, data):
        self.get_logger().info('Received image')

        # TODO (write your code here)
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

        cali = Calibration()
        cali.calibrate_extrinsic(img)

        # To stack up the objects
        arm = ArmControl(send_script=send_script, set_io=set_io)
        gripper = Gripper(arm)
        gripper.run()


def send_script(script):
    arm_node = rclpy.create_node('arm')
    arm_cli = arm_node.create_client(SendScript, 'send_script')

    while not arm_cli.wait_for_service(timeout_sec=1.0):
        arm_node.get_logger().info('service not availabe, waiting again...')

    move_cmd = SendScript.Request()
    move_cmd.script = script
    arm_cli.call_async(move_cmd)
    arm_node.destroy_node()


def set_io(state):
    gripper_node = rclpy.create_node('gripper')
    gripper_cli = gripper_node.create_client(SetIO, 'set_io')

    while not gripper_cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not availabe, waiting again...')

    io_cmd = SetIO.Request()
    io_cmd.module = 1
    io_cmd.type = 1
    io_cmd.pin = 0
    io_cmd.state = state
    gripper_cli.call_async(io_cmd)
    gripper_node.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImageSub('image_sub')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
