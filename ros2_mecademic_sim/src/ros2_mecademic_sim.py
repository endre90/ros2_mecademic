import sys
import rclpy
import time
import csv
import os
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import threading
from ament_index_python.packages import get_package_share_directory

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

class Window(QWidget):
    def __init__(self, parent=None):
        super(Window, self).__init__(parent)

        self.joint_names = ["meca_axis_1_joint", "meca_axis_2_joint", "meca_axis_3_joint", 
            "meca_axis_4_joint", "meca_axis_5_joint", "meca_axis_6_joint"]

        self.active_button = False

        minimum = -180
        maximum = 180
        step = 1

        grid = QGridLayout()
        i = 0

        for name in self.joint_names:
            slider1Box = QGroupBox(name)
            slider1 = QSlider(Qt.Horizontal)
            slider1.setFocusPolicy(Qt.StrongFocus)
            slider1.setTickPosition(QSlider.TicksBothSides)
            slider1.setMinimum(minimum)
            slider1.setMaximum(maximum)
            slider1.setSingleStep(step)
            vbox1 = QVBoxLayout()
            vbox1.addWidget(slider1)
            vbox1.addStretch(1)
            slider1Box.setLayout(vbox1)

            label1Box = QGroupBox("actual")
            label1 = QLabel("value")
            slider1.valueChanged.connect(label1.setNum)
            vbox1l = QVBoxLayout()
            vbox1l.addWidget(label1)
            vbox1l.addStretch(1)
            label1Box.setLayout(vbox1l)
        
            grid.addWidget(slider1Box, i, 0)
            grid.addWidget(label1Box, i, 1)
            i = i + 1

        radio1 = QRadioButton("Active")
        radio1.setChecked(False)
        grid.addWidget(radio1)

        self.setLayout(grid)

        self.setWindowTitle("Meca 500 joint pose")
        self.resize(450, 300)

class Meca500R3Sim(Node):

    def __init__(self):
        super().__init__("ros2_mecademic_sim")

        self.from_sp = String()
        self.to_sp = String()
        self.from_r = JointState()
        self.to_r = JointState()

        self.joint_tolerance = 0.05
        self.tmr_period = 0.02
        self.tmr_period2 = 0.5

        self.joint_names = ["meca_axis_1_joint", "meca_axis_2_joint", "meca_axis_3_joint", 
            "meca_axis_4_joint", "meca_axis_5_joint", "meca_axis_6_joint"]

        self.joints_input = os.path.join(get_package_share_directory('ros2_mecademic_utils'),
            'poses', 'joint_poses.csv')

        self.act_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.ref_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.pub_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.act_pos_str = ""
        self.ref_pos_str = ""

        # Could be good to start the subscribers first so that they update the variables if other nodes are up
        self.joint_subscriber = self.create_subscription(
            JointState, 
            "/meca_500_joint_states",
            self.joint_callback,
            10)

        # Could be good to start the subscribers first so that they update the variables if other nodes are up
        self.cmd_subscriber = self.create_subscription(
            String, 
            "/meca_500_sp_to_interfacer",
            self.sp_callback,
            10)

        # Then sleep for a bit so that the node get the updated variables before publishing them
        time.sleep(2)

        self.state_to_sp_publisher_ = self.create_publisher(
            String,
            "/meca_500_interfacer_to_sp",
            10)
        
        self.joint_cmd_publisher_ = self.create_publisher(
            JointState,
            "/meca_500_joint_states",
            10)
    
        # Decouple message receiving and forwarding
        self.interfacer_to_sp_tmr = self.create_timer(
            self.tmr_period2, 
            self.interfacer_to_sp_publisher_callback)

        self.interfacer_to_r_tmr = self.create_timer(
            self.tmr_period, 
            self.interfacer_to_r_publisher_callback)

    def joint_callback(self, data):
        self.act_pos[0] = data.position[0]
        self.act_pos[1] = data.position[1]
        self.act_pos[2] = data.position[2]
        self.act_pos[3] = data.position[3]
        self.act_pos[4] = data.position[4]
        self.act_pos[5] = data.position[5]

    def sp_callback(self, data):
        self.ref_pos = self.find_pose(data.data, self.joints_input)
        print(self.ref_pos)

    def interfacer_to_r_publisher_callback(self):
        for i in range(0, 6):
            if self.ref_pos != None:
                if self.ref_pos[i] < self.act_pos[i]:
                    self.pub_pos[i] = round(self.act_pos[i] - 0.01, 2)
                elif self.ref_pos[i] > self.act_pos[i]:
                    self.pub_pos[i] = round(self.act_pos[i] + 0.01, 2)
                else:
                    pass
            else:
                pass
        self.to_r.name = self.joint_names
        self.to_r.position = self.pub_pos
        self.joint_cmd_publisher_.publish(self.to_r)
        
    def interfacer_to_sp_publisher_callback(self):
        self.to_sp.data = self.get_static_joint_pose()
        self.state_to_sp_publisher_.publish(self.to_sp)

    def find_pose(self, name, input_f):
        '''
        Returns the saved pose that matches the pose name
        '''

        pose = []
        with open(input_f, 'r') as f_in:
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
            pass
            # self.pose_name_error = 'pose with the name ' + name + ' not saved'
            # return []

    def get_static_joint_pose(self):
        '''
        Returns the saved pose name that matches the pose
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

    

def main(args=None):

    def launch_robot():
        def launch_robot_callback_local():
            rclpy.init(args=args)
            ros2_mecademic_sim = Meca500R3Sim()
            rclpy.spin(ros2_mecademic_sim)
            ros2_mecademic_sim.destroy_node()
            rclpy.shutdown()
        t = threading.Thread(target=launch_robot_callback_local)
        t.daemon = True
        t.start()
    
    # Window has to be in the main thread
    def launch_window():
        app = QApplication(sys.argv)
        clock = Window()
        clock.show()
        sys.exit(app.exec_())

    launch_robot()    
    launch_window()
    

if __name__ == '__main__':
    main()