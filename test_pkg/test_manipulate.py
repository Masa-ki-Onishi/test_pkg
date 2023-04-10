import math
import time
import threading
import numpy
# ROS2
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller_node')
        self.joint_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.joint_names = [
        'joint1',
        'joint2',
        'joint3',
        'joint4',
        'gripper',
        ]

    def deg_to_rad(self, deg):
        return math.radians(deg)

    def inverse_kinematics(self, coordinate):
        x = coordinate[0]
        y = coordinate[1]
        l0 = 0
        l1 = 0.128
        l2 = 0.124
        l3 = 0.126
        x -= l3
        y -= l0

        try:
            shoulder_angle = -1*math.acos((x*x+y*y+l1*l1-l2*l2) / (2*l1*math.sqrt(x*x+y*y))) + math.atan(y/x)
            elbow_angle = math.atan((y-l1*math.sin(shoulder_angle))/(x-l1*math.cos(shoulder_angle)))-shoulder_angle - math.pi/2
            wrist_angle = -1*(shoulder_angle + elbow_angle) - math.pi/2
            angle_list = list(map(math.degrees, [0.0, shoulder_angle, elbow_angle, wrist_angle, 0.0]))
            print(angle_list)
            return angle_list
        except ValueError:
            self.get_logger().info('Can not move arm.')
            return [numpy.nan]*5

    def publish_joint(self, joint_angle, execute_time=2):
        self.get_logger().info('Publish joint.')
        joint_angle = list(map(self.deg_to_rad, joint_angle))
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.joint_names
        msg.points = [JointTrajectoryPoint()]
        msg.points[0].positions = list(map(float, joint_angle))
        msg.points[0].time_from_start = Duration(
                seconds=int(execute_time), nanoseconds=(execute_time-int(execute_time))*1e9).to_msg()
        self.joint_pub.publish(msg)

    def manipulation(self, coordinate, gripper=False):
        angle_list = self.inverse_kinematics(coordinate)
        if gripper:
            angle_list[4] = 55
        else:
            angle_list[4] = -90
        self.publish_joint(angle_list, 2)

    def start_up(self):
        angle_list = [0.0, -90, 90, 0.0, -90]
        self.publish_joint(angle_list)


def main():
    rclpy.init()
    node = JointController()

    thread = threading.Thread(target=rclpy.spin, args=(node,))
    thread.start()

    time.sleep(0.5)
    
    node.start_up()
    time.sleep(3.0)
    #joint = [0.0, 0.0, 0.0, 0.0, -90] #gripper >>> min: -90, max: 55
    #node.publish_joint(joint, 2)
    #time.sleep(3.0)
    node.manipulation([0.25, 0.0])
    time.sleep(3.0)
    node.manipulation([0.25, -0.1], True)
    time.sleep(3.0)
    node.manipulation([0.25, 0.05])
    time.sleep(3.0)
    node.manipulation([0.25, 0.0])
    time.sleep(3.0)
    node.manipulation([0.378, 0.0])
    time.sleep(3.0)
    node.manipulation([0.25, 0.0])
    time.sleep(3.0)
    joint = [0.0, 0.0, 0.0, 0.0, 55]
    node.publish_joint(joint, 2)
    time.sleep(3.0)
    node.start_up()
    #node.publish_gripper(gripper, execute_time)

    rclpy.shutdown()
