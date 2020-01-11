import sys
import rclpy
import time
import csv
import os
import math
from rclpy.node import Node
from sensor_msgs.msg import JointState
from ros2_mecademic_msgs.msg import MecademicGuiToEsd
from ros2_mecademic_msgs.msg import MecademicEsdToGui
from ros2_mecademic_msgs.msg import MecademicSPToEsd
from ros2_mecademic_msgs.msg import MecademicEsdToSP
from ros2_mecademic_msgs.msg import MecademicUtilsToEsd
from ros2_mecademic_msgs.msg import MecademicEsdToUtils

class Ros2MecademicSimulator(Node):

    def __init__(self):
        super().__init__("ros2_mecademic_simulator")

        self.esd_to_gui_msg = MecademicEsdToGui()
        self.esd_to_utils_msg = MecademicEsdToUtils()
        self.joint_state = JointState()
    
        self.joint_state_timer_period = 0.005
        self.esd_to_gui_timer_period = 0.1
        self.esd_to_utils_timer_period = 0.1

        self.gui_control_enabled = False
        self.gui_speed_control = 0
        self.gui_joint_control = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.utility_action = ""
        self.utility_pose_name = ""

        self.joint_names = ["meca_axis_1_joint", "meca_axis_2_joint", "meca_axis_3_joint", 
            "meca_axis_4_joint", "meca_axis_5_joint", "meca_axis_6_joint"]

        self.act_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.pub_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.act_pos_str = ""
        self.ref_pos_str = ""

        self.joint_state_subscriber = self.create_subscription(
            JointState, 
            "/mecademic_joint_states",
            self.joint_state_callback,
            10)

        self.gui_to_esd_subscriber = self.create_subscription(
            MecademicGuiToEsd, 
            "/mecademic_gui_to_esd",
            self.gui_to_esd_callback,
            10)

        time.sleep(2)

        self.esd_to_gui_publisher_ = self.create_publisher(
            MecademicEsdToGui,
            "/mecademic_esd_to_gui",
            10)
        
        self.esd_to_utils_publisher_ = self.create_publisher(
            MecademicEsdToUtils,
            "/mecademic_esd_to_utils",
            10)
        
        self.joint_state_publisher_ = self.create_publisher(
            JointState,
            "/mecademic_joint_states",
            10)
    
        self.esd_to_gui_timer = self.create_timer(
            self.esd_to_gui_timer_period, 
            self.esd_to_gui_publisher_callback)

        self.joint_state_publisher_timer = self.create_timer(
            self.joint_state_timer_period, 
            self.joint_state_publisher_callback)
        
        self.esd_to_utils_publisher_timer = self.create_timer(
            self.esd_to_utils_timer_period, 
            self.esd_to_utils_publisher_callback)

    def joint_state_callback(self, data):
        self.act_pos[0] = data.position[0]
        self.act_pos[1] = data.position[1]
        self.act_pos[2] = data.position[2]
        self.act_pos[3] = data.position[3]
        self.act_pos[4] = data.position[4]
        self.act_pos[5] = data.position[5]

    # def sp_callback(self, data):
    #     self.ref_pos = self.find_pose(data.data, self.joints_input)
    #     print(self.ref_pos)

    def gui_to_esd_callback(self, data):
        self.gui_control_enabled = data.gui_control_enabled
        self.gui_speed_control = data.gui_speed_control
        self.gui_joint_control[0] = round(data.gui_joint_control[0], 2)
        self.gui_joint_control[1] = round(data.gui_joint_control[1], 2)
        self.gui_joint_control[2] = round(data.gui_joint_control[2], 2)
        self.gui_joint_control[3] = round(data.gui_joint_control[3], 2)
        self.gui_joint_control[4] = round(data.gui_joint_control[4], 2)
        self.gui_joint_control[5] = round(data.gui_joint_control[5], 2)
        self.utility_action = data.utility_action
        self.utility_pose_name = data.utility_pose_name

    def joint_state_publisher_callback(self):        
        if self.gui_control_enabled == True:
            for i in range(0, 6):
                if self.gui_joint_control != None:
                    if self.gui_joint_control[i] < self.act_pos[i]:
                        if self.gui_joint_control[i] < self.act_pos[i] - 0.0001*self.gui_speed_control:
                            self.pub_pos[i] = round(self.act_pos[i] - 0.0001*self.gui_speed_control, 4)
                        else:
                            self.pub_pos[i] = round(self.act_pos[i] - 0.001, 3)
                    elif self.gui_joint_control[i] > self.act_pos[i]:
                        if self.gui_joint_control[i] > self.act_pos[i] + 0.0001*self.gui_speed_control:
                            self.pub_pos[i] = round(self.act_pos[i] + 0.0001*self.gui_speed_control, 4)
                        else:
                            self.pub_pos[i] = round(self.act_pos[i] + 0.001, 3)
                    else:
                        pass
                else:
                    pass
        else:
            pass

        self.joint_state.name = self.joint_names
        self.joint_state.position = self.pub_pos
        self.joint_state_publisher_.publish(self.joint_state)
            
    def esd_to_gui_publisher_callback(self):
        pass

    def esd_to_utils_publisher_callback(self):
        self.esd_to_utils_msg.utility_action = self.utility_action
        self.esd_to_utils_msg.utility_pose_name = self.utility_pose_name
        self.esd_to_utils_publisher_.publish(self.esd_to_utils_msg)

def main(args=None):

    rclpy.init(args=args)
    ros2_mecademic_simulator = Ros2MecademicSimulator()
    rclpy.spin(ros2_mecademic_simulator)
    ros2_mecademic_simulator.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()