import sys
import rclpy
import time
import csv
import os
import math
import numpy
import ast
from rclpy.node import Node
from sensor_msgs.msg import JointState
from ros2_mecademic_msgs.msg import MecademicGuiToEsd
from ros2_mecademic_msgs.msg import MecademicEsdToGui
from ros2_mecademic_msgs.msg import MecademicSPToEsd
from ros2_mecademic_msgs.msg import MecademicEsdToSP
from ament_index_python.packages import get_package_share_directory

class Ros2MecademicSimulator(Node):

    def __init__(self):
        super().__init__("ros2_mecademic_simulator")

        self.act_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.pub_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_tolerance = 0

        self.joints_input = os.path.join(get_package_share_directory('ros2_mecademic_utilities'),
            'poses', 'joint_poses.csv')

        # gui to esd:
        self.gui_to_esd_msg = MecademicGuiToEsd()
        self.gui_to_esd_msg.gui_control_enabled = False                       
        self.gui_to_esd_msg.gui_speed_control = 0                             
        self.gui_to_esd_msg.gui_joint_control = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]                      

        self.gui_to_esd_subscriber = self.create_subscription(
            MecademicGuiToEsd, 
            "/mecademic_gui_to_esd",
            self.gui_to_esd_callback,
            10)
        
        # joints to esd:
        self.joint_state_subscriber = self.create_subscription(
            JointState, 
            "/mecademic_joint_states",
            self.joint_state_callback,
            10)

        time.sleep(2)

        # esd to gui:
        self.esd_to_gui_msg = MecademicEsdToGui()
        self.esd_to_gui_msg.actual_pose = "init"
        self.esd_to_gui_timer_period = 0.1

        self.esd_to_gui_publisher_ = self.create_publisher(
            MecademicEsdToGui,
            "/mecademic_esd_to_gui",
            10)

        self.esd_to_gui_timer = self.create_timer(
            self.esd_to_gui_timer_period, 
            self.esd_to_gui_publisher_callback)

        # esd to joints:
        self.joint_state = JointState()
        self.joint_names = ["meca_axis_1_joint", "meca_axis_2_joint", "meca_axis_3_joint", 
            "meca_axis_4_joint", "meca_axis_5_joint", "meca_axis_6_joint"]

        self.joint_state_timer_period = 0.005

        self.joint_state_publisher_ = self.create_publisher(
            JointState,
            "/mecademic_joint_states",
            10)
    
        self.joint_state_publisher_timer = self.create_timer(
            self.joint_state_timer_period, 
            self.joint_state_publisher_callback)

    def get_pose_from_pose_name(self, name):
        '''
        Returns the saved pose that matches the pose name
        '''

        pose = []
        with open(self.joints_input, 'r') as f_in:
            csv_reader = csv.reader(f_in, delimiter=':')
            for row in csv_reader:
                if name == row[0]:
                    pose = ast.literal_eval(row[1])
                    break
                else:
                    pass

        if pose != []:
            self.pose_name_error = ''
            return pose
        else:
            self.pose_name_error = 'pose with the name ' + name + ' not saved'
            return []

    def get_pose_name_from_pose(self):
        '''
        While all joint velocities are 0, compare current robot joint pose with saved poses in the joint_csv file
        and return the name of the saved pose if they match with a tolerance, else return unknown as the 
        current joint pose. Using joint_callback because it is very slow to get_current_joint_values via 
        moveit_commander according to KCacheGrind.
        '''

        actual_joint_pose = ""
        current_pose = self.act_pos

        with open(self.joints_input, 'r') as joint_csv:
            joint_csv_reader = csv.reader(joint_csv, delimiter=':')
            for row in joint_csv_reader:
                if len(ast.literal_eval(row[1])) == 6 and current_pose != []:
                    saved_pose = ast.literal_eval(row[1])
                    if all(numpy.isclose(current_pose[i], saved_pose[i], atol=self.joint_tolerance) for i in range(0, 6)):
                        actual_joint_pose = row[0]
                        break
                    else:
                        actual_joint_pose = "UNKNOWN"
                        pass
                else:
                    pass
        
        return actual_joint_pose

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
        self.gui_to_esd_msg.gui_control_enabled = data.gui_control_enabled
        self.gui_to_esd_msg.gui_speed_control = data.gui_speed_control
        self.gui_to_esd_msg.gui_joint_control[0] = round(data.gui_joint_control[0], 2)
        self.gui_to_esd_msg.gui_joint_control[1] = round(data.gui_joint_control[1], 2)
        self.gui_to_esd_msg.gui_joint_control[2] = round(data.gui_joint_control[2], 2)
        self.gui_to_esd_msg.gui_joint_control[3] = round(data.gui_joint_control[3], 2)
        self.gui_to_esd_msg.gui_joint_control[4] = round(data.gui_joint_control[4], 2)
        self.gui_to_esd_msg.gui_joint_control[5] = round(data.gui_joint_control[5], 2)
        # self.gui_to_esd_msg.utility_action = data.utility_action
        # self.gui_to_esd_msg.utility_pose_name = data.utility_pose_name

    def joint_state_publisher_callback(self):        
        if self.gui_to_esd_msg.gui_control_enabled == True:
            for i in range(0, 6):
                if self.gui_to_esd_msg.gui_joint_control != None:
                    if self.gui_to_esd_msg.gui_joint_control[i] < self.act_pos[i] - 0.01:
                        # self.pub_pos[i] = self.act_pos[i] - 0.001
                        if self.gui_to_esd_msg.gui_joint_control[i] < self.act_pos[i] - 0.0001*self.gui_to_esd_msg.gui_speed_control:
                            self.pub_pos[i] = round(self.act_pos[i] - 0.0001*self.gui_to_esd_msg.gui_speed_control, 4)
                        else:
                            self.pub_pos[i] = round(self.act_pos[i] - 0.001, 3)
                    elif self.gui_to_esd_msg.gui_joint_control[i] > self.act_pos[i] + 0.01:
                        if self.gui_to_esd_msg.gui_joint_control[i] > self.act_pos[i] + 0.0001*self.gui_to_esd_msg.gui_speed_control:
                            self.pub_pos[i] = round(self.act_pos[i] + 0.0001*self.gui_to_esd_msg.gui_speed_control, 4)
                        else:
                            self.pub_pos[i] = round(self.act_pos[i] + 0.001, 3)
                        # self.pub_pos[i] = self.act_pos[i] + 0.001
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
        self.esd_to_gui_msg.actual_pose = self.get_pose_name_from_pose()
        self.esd_to_gui_publisher_.publish(self.esd_to_gui_msg)
    
def main(args=None):

    rclpy.init(args=args)
    ros2_mecademic_simulator = Ros2MecademicSimulator()
    rclpy.spin(ros2_mecademic_simulator)
    ros2_mecademic_simulator.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()