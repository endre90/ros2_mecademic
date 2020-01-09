import sys
import rclpy
import time
import csv
import os
import math
from rclpy.node import Node
from std_msgs.msg import String
from ros2_mecademic_msgs.msg import MecademicInterfacerToDriver
from sensor_msgs.msg import JointState

class Ros2MecademicSimulator(Node):

    def __init__(self):
        super().__init__("ros2_mecademic_simulator")

        self.interfacer_to_driver = MecademicInterfacerToDriver()
        self.joint_state = JointState()
    
        self.joint_state_timer_period = 0.005

        self.gui_control_enabled = False
        self.gui_speed_control = 0
        self.gui_joint_control = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.joint_names = ["meca_axis_1_joint", "meca_axis_2_joint", "meca_axis_3_joint", 
            "meca_axis_4_joint", "meca_axis_5_joint", "meca_axis_6_joint"]

        self.act_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.pub_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.act_pos_str = ""
        self.ref_pos_str = ""

        # # Could be good to start the subscribers first so that they update the variables if other nodes are up
        self.joint_state_subscriber = self.create_subscription(
            JointState, 
            "/mecademic_joint_states",
            self.joint_callback,
            10)

        # Could be good to start the subscribers first so that they update the variables if other nodes are up
        self.interfacer_to_driver_subscriber = self.create_subscription(
            MecademicInterfacerToDriver, 
            "/mecademic_interfacer_to_driver",
            self.interfacer_to_driver_callback,
            10)

        # Then sleep for a bit so that the node get the updated variables before publishing them
        time.sleep(2)

        # self.state_to_sp_publisher_ = self.create_publisher(
        #     String,
        #     "/meca_500_interfacer_to_sp",
        #     10)
        
        self.joint_state_publisher_ = self.create_publisher(
            JointState,
            "/mecademic_joint_states",
            10)
    
        # Decouple message receiving and forwarding
        self.interfacer_to_r_tmr = self.create_timer(
            self.joint_state_timer_period, 
            self.joint_state_publisher_callback)

    def joint_callback(self, data):
        self.act_pos[0] = data.position[0]
        self.act_pos[1] = data.position[1]
        self.act_pos[2] = data.position[2]
        self.act_pos[3] = data.position[3]
        self.act_pos[4] = data.position[4]
        self.act_pos[5] = data.position[5]

    # def sp_callback(self, data):
    #     self.ref_pos = self.find_pose(data.data, self.joints_input)
    #     print(self.ref_pos)

    def interfacer_to_driver_callback(self, data):
        self.gui_control_enabled = data.gui_control_enabled
        self.gui_speed_control = data.gui_speed_control
        self.gui_joint_control[0] = round(data.gui_joint_control[0], 2)
        self.gui_joint_control[1] = round(data.gui_joint_control[1], 2)
        self.gui_joint_control[2] = round(data.gui_joint_control[2], 2)
        self.gui_joint_control[3] = round(data.gui_joint_control[3], 2)
        self.gui_joint_control[4] = round(data.gui_joint_control[4], 2)
        self.gui_joint_control[5] = round(data.gui_joint_control[5], 2)

    def joint_state_publisher_callback(self):

        # self.publish_rate = self.joint_state_timer_period
        # if self.gui_speed_control != 0:
        #     self.joint_state_timer_period = 0.005*(1/self.gui_speed_control)
        
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
            

    # def interfacer_to_r_publisher_callback(self):
    #     if CommVariables.gui_control_enabled == False:
    #         for i in range(0, 6):
    #             if self.ref_pos != None:
    #                 if self.ref_pos[i] < self.act_pos[i]:
    #                     self.pub_pos[i] = round(self.act_pos[i] - 0.01, 2)
    #                 elif self.ref_pos[i] > self.act_pos[i]:
    #                     self.pub_pos[i] = round(self.act_pos[i] + 0.01, 2)
    #                 else:
    #                     pass
    #             else:
    #                 pass
    #     else:
    #     # print(CommVariables.slider_1_value)
    #         go_to_pos = [round(CommVariables.slider_1_value * math.pi / 180, 2),
    #             round(CommVariables.slider_2_value * math.pi / 180, 2),
    #             round(CommVariables.slider_3_value * math.pi / 180, 2),
    #             round(CommVariables.slider_4_value * math.pi / 180, 2),
    #             round(CommVariables.slider_5_value * math.pi / 180, 2),
    #             round(CommVariables.slider_6_value * math.pi / 180, 2)]

    #         for i in range(0, 6):
    #             if go_to_pos != None:
    #                 if go_to_pos[i] < self.act_pos[i]:
    #                     self.pub_pos[i] = round(self.act_pos[i] - 0.01, 2)
    #                 elif go_to_pos[i] > self.act_pos[i]:
    #                     self.pub_pos[i] = round(self.act_pos[i] + 0.01, 2)
    #                 else:
    #                     pass

    #         self.ref_pos = go_to_pos

        # self.to_r.name = self.joint_names
        # self.to_r.position = self.pub_pos
        # self.joint_cmd_publisher_.publish(self.to_r)
        
    # def interfacer_to_sp_publisher_callback(self):
    #     print(CommVariables.gui_control_enabled)
    #     self.to_sp.data = self.get_static_joint_pose()
    #     self.state_to_sp_publisher_.publish(self.to_sp)

    # def find_pose(self, name, input_f):
    #     '''
    #     Returns the saved pose that matches the pose name
    #     '''

    #     pose = []
    #     with open(input_f, 'r') as f_in:
    #         csv_reader = csv.reader(f_in, delimiter=':')
    #         for row in csv_reader:
    #             if name == row[0]:
    #                 pose = ast.literal_eval(row[1])
    #                 break
    #             else:
    #                 pass

    #     if pose != []:
    #         self.pose_name_error = ''
    #         return pose
    #     else:
    #         pass
    #         # self.pose_name_error = 'pose with the name ' + name + ' not saved'
    #         # return []

    # def get_static_joint_pose(self):
    #     '''
    #     Returns the saved pose name that matches the pose
    #     '''

    #     actual_joint_pose = ""
    #     current_pose = self.act_pos

    #     with open(self.joints_input, 'r') as joint_csv:
    #         joint_csv_reader = csv.reader(joint_csv, delimiter=':')
    #         for row in joint_csv_reader:
    #             if len(ast.literal_eval(row[1])) == 6 and current_pose != []:
    #                 saved_pose = ast.literal_eval(row[1])
    #                 if all(numpy.isclose(current_pose[i], saved_pose[i], atol=self.joint_tolerance) for i in range(0, 6)):
    #                     actual_joint_pose = row[0]
    #                     break
    #                 else:
    #                     actual_joint_pose = "UNKNOWN"
    #                     pass
    #             else:
    #                 pass
        
    #     return actual_joint_pose

def main(args=None):

    rclpy.init(args=args)
    ros2_mecademic_simulator = Ros2MecademicSimulator()
    rclpy.spin(ros2_mecademic_simulator)
    ros2_mecademic_simulator.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()