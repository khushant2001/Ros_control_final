import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import DockStatus
from irobot_create_msgs.msg import HazardDetectionVector as detect
from irobot_create_msgs.msg import IrIntensityVector as ir
from nav_msgs.msg import Odometry 
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np
from sensor_msgs.msg import Imu
import math
import time
class move(Node):
    def __init__(self):
        super().__init__("Moving")
        self.move = self.create_publisher(Twist, "/robot_1/cmd_vel",1)
        self.timer = self.create_timer(.05, self.move_func)
        qos_profil = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.i = 0
        self.initial_position = np.array([[0,0]])
        self.haz_sub = self.create_subscription(detect, "/robot_1/hazard_detection", self.hazard_detect,qos_profile = qos_profil)
        self.hazard_status = None
        self.ir_sub = self.create_subscription(ir, "/robot_1/ir_intensity", self.ir_sense, qos_profile = qos_profil)
        self.ir_status = [0,0,0,0,0,0,0]
        #self.imu_sub = self.create_subscription(Imu, "/robot_1/imu", self.imu_sense, qos_profile = qos_profil)
        self.imu_status = np.array([[0,0]])
        self.imu_sub = self.create_subscription(Odometry, "/robot_1/odom", self.imu_sense, qos_profile = qos_profil)

    def imu_sense(self, imu_msg:Odometry):
        #self.get_logger().info("Getting Odometry")
        self.imu_status = np.array([
            [math.floor(1*imu_msg.pose.pose.position._x), math.floor(1*imu_msg.pose.pose.position._y)]]) 
        if self.i == 0:
            self.initial_position[0,0] = self.imu_status[0,0]
            self.initial_position[0,1] = self.imu_status[0,1]
            self.i+=1

    def ir_sense(self,ir_msg:ir):
        #self.get_logger().info("Value = ")
        self.ir_status[0] = ir_msg.readings[0].value
        self.ir_status[1] = ir_msg.readings[1].value
        self.ir_status[2] = ir_msg.readings[2].value
        self.ir_status[3] = ir_msg.readings[3].value
        self.ir_status[4] = ir_msg.readings[4].value
        self.ir_status[5] = ir_msg.readings[5].value
        self.ir_status[6] = ir_msg.readings[6].value
    
    def hazard_detect(self,hazmsg:detect):
        #self.get_logger().info("Checking for hazards")
        if len(hazmsg.detections) > 0:
            for detection in hazmsg.detections:
                if detection.header.frame_id != 'base_link':
                    self.hazard_status = detection.header.frame_id  
        else:
            self.hazard_status = None
        print(self.hazard_status)
    def check_dock(self,msg:DockStatus):
        #self.get_logger().info("Checking for docking")
        self.dock_status = msg.is_docked

    def move_func(self):
        msg = Twist()
        if self.hazard_status == 'bump_right' or self.hazard_status == 'bump_front_right':
            print("hi")
            vel = .5
            while True:
                if vel < 0:
                    time.sleep(2)
                    while True:
                        if max(self.ir_status) > 20:
                            break
                        msg.linear.x = .2
                        msg.angular.z = .3
                        self.move.publish(msg)
                    break
                msg.linear.x = vel
                self.move.publish(msg)
                if max(self.ir_status[0:3])>20:
                    print("close")
                    msg.linear.x = 0.0
                    msg.angular.z = -0.6
                    self.move.publish(msg)
                elif max(self.ir_status[4:6])>20:
                    print("close")
                    msg.linear.x = 0.0
                    msg.angular.z = 0.6
                    self.move.publish(msg)
                vel = vel - .00000005

        elif self.hazard_status== 'bump_front_center':
            vel = 0.75
            while True:
                if vel < 0:
                    time.sleep(2)
                    while True:
                        if max(self.ir_status) > 20:
                            break
                        msg.linear.x = .2
                        msg.angular.z = .3
                        self.move.publish(msg)
                    break
                msg.linear.x = vel
                self.move.publish(msg)
                if max(self.ir_status[0:3])>20:
                    msg.linear.x = 0.0
                    msg.angular.z = -0.6
                    self.move.publish(msg)
                elif max(self.ir_status[4:6])>20:
                    msg.linear.x = 0.0
                    msg.angular.z = 0.6
                    self.move.publish(msg)
                vel = vel-.00000005
        elif self.hazard_status == 'bump_left' or self.hazard_status == 'bump_front_left':
            vel = 1.0
            while True:
                if vel < 0:
                    time.sleep(2)
                    while True:
                        if max(self.ir_status) > 20:
                            break
                        msg.linear.x = .2
                        msg.angular.z = .3
                        self.move.publish(msg)
                    break
                msg.linear.x = vel
                self.move.publish(msg)
                if max(self.ir_status[0:4])>20:
                    msg.linear.x = 0.0
                    msg.angular.z = -0.6
                    self.move.publish(msg)
                elif max(self.ir_status[4:6])>20:
                    msg.linear.x = 0.0
                    msg.angular.z = 0.6
                    self.move.publish(msg)
                vel = vel-.00000005
def main(args = None):
    rclpy.init(args = args)
    node = move()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == 'main':
    main()