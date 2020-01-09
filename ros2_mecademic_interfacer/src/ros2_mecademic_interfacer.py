import sys
import rclpy
import time
import csv
import os
import math
from rclpy.node import Node
from std_msgs.msg import String
from ros2_mecademic_msgs.msg import MecademicInterfacerToDriver
from ros2_mecademic_msgs.msg import MecademicGuiToInterfacer
# from ros2_mecademic_msgs.msg import MecademicSimulatorToInterfacer
# from ros2_mecademic_msgs.msg import MecademicDriverToInterfacer
from sensor_msgs.msg import JointState

class Ros2MecademicInterfacer(Node):

    def __init__(self):
        super().__init__("ros2_mecademic_interfacer")

        self.gui_to_interfacer = MecademicGuiToInterfacer()
        self.interfacer_to_driver = MecademicInterfacerToDriver()
        # self.interfacer_to_driver = MecademicInterfacerToDriver()
        # self.simulator_to_interfacer = MecademicSimulatorToInterfacer()
        # self.driver_to_interfacer = MecademicDriverToInterfacer()

        self.interfacer_to_driver.gui_control_enabled = False
        self.gui_to_interfacer.gui_speed_control = 0
        self.interfacer_to_driver.gui_joint_control = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.interfacer_to_driver_timer_period = 0.05
    
        self.gui_to_interfacer_subscriber = self.create_subscription(
            MecademicGuiToInterfacer, 
            "/mecademic_gui_to_interfacer",
            self.gui_to_interfacer_callback,
            10)

        # Then sleep for a bit so that the node gets the updated variables before publishing them
        time.sleep(2)

        self.interfacer_to_driver_publisher_ = self.create_publisher(
            MecademicInterfacerToDriver,
            "/mecademic_interfacer_to_driver",
            10)

        # Decouple message receiving and forwarding
        self.interfacer_to_driver_timer = self.create_timer(
            self.interfacer_to_driver_timer_period, 
            self.interfacer_to_driver_callback)

    def interfacer_to_driver_callback(self):
        self.interfacer_to_driver_publisher_.publish(self.interfacer_to_driver)

    def gui_to_interfacer_callback(self, data):
        self.interfacer_to_driver.gui_control_enabled = data.gui_control_enabled
        self.interfacer_to_driver.gui_speed_control = data.gui_speed_control
        self.interfacer_to_driver.gui_joint_control = data.gui_joint_control

def main(args=None):
    rclpy.init(args=args)
    ros2_mecademic_interfacer = Ros2MecademicInterfacer()
    rclpy.spin(ros2_mecademic_interfacer)
    ros2_mecademic_interfacer.destroy_node()
    rclpy.shutdown()
            
if __name__ == '__main__':
    main()