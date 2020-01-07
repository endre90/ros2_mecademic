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


        # Sliders setup:
        self.slider_box_1 = QGroupBox(self.joint_names[0])
        self.slider_box_2 = QGroupBox(self.joint_names[1])
        self.slider_box_3 = QGroupBox(self.joint_names[2])
        self.slider_box_4 = QGroupBox(self.joint_names[3])
        self.slider_box_5 = QGroupBox(self.joint_names[4])
        self.slider_box_6 = QGroupBox(self.joint_names[5])

        self.slider_boxes = [self.slider_box_1, self.slider_box_2, self.slider_box_3, self.slider_box_4, self.slider_box_5, self.slider_box_6]

        self.slider_1 = QSlider(Qt.Horizontal)
        self.slider_2 = QSlider(Qt.Horizontal)
        self.slider_3 = QSlider(Qt.Horizontal)
        self.slider_4 = QSlider(Qt.Horizontal)
        self.slider_5 = QSlider(Qt.Horizontal)
        self.slider_6 = QSlider(Qt.Horizontal)

        self.sliders = [self.slider_1, self.slider_2, self.slider_3, self.slider_4, self.slider_5, self.slider_6]
        
        self.slider_box_1_layout = QVBoxLayout()
        self.slider_box_2_layout = QVBoxLayout()
        self.slider_box_3_layout = QVBoxLayout()
        self.slider_box_4_layout = QVBoxLayout()
        self.slider_box_5_layout = QVBoxLayout()
        self.slider_box_6_layout = QVBoxLayout()

        for slider in self.sliders:
            slider.setFocusPolicy(Qt.StrongFocus)
            slider.setTickPosition(QSlider.TicksBothSides)
            slider.setMinimum(minimum)
            slider.setMaximum(maximum)
            slider.setSingleStep(step)
            slider.setMinimumWidth(300)
        
        self.slider_box_1_layout.addWidget(self.slider_1)
        self.slider_box_2_layout.addWidget(self.slider_2)
        self.slider_box_3_layout.addWidget(self.slider_3)
        self.slider_box_4_layout.addWidget(self.slider_4)
        self.slider_box_5_layout.addWidget(self.slider_5)
        self.slider_box_6_layout.addWidget(self.slider_6)

        self.slider_box_1.setLayout(self.slider_box_1_layout)
        self.slider_box_2.setLayout(self.slider_box_2_layout)
        self.slider_box_3.setLayout(self.slider_box_3_layout)
        self.slider_box_4.setLayout(self.slider_box_4_layout)
        self.slider_box_5.setLayout(self.slider_box_5_layout)
        self.slider_box_6.setLayout(self.slider_box_6_layout)


        # Indicator labels setup:
        self.label_box_1 = QGroupBox("measured")
        self.label_box_2 = QGroupBox("measured")
        self.label_box_3 = QGroupBox("measured")
        self.label_box_4 = QGroupBox("measured")
        self.label_box_5 = QGroupBox("measured")
        self.label_box_6 = QGroupBox("measured")

        self.label_boxes = [self.label_box_1, self.label_box_2, self.label_box_3, self.label_box_4, self.label_box_5, self.label_box_6]

        self.label_1 = QLabel("value")
        self.label_2 = QLabel("value")
        self.label_3 = QLabel("value")
        self.label_4 = QLabel("value")
        self.label_5 = QLabel("value")
        self.label_6 = QLabel("value")

        self.labels = [self.label_1, self.label_2, self.label_3, self.label_4, self.label_5, self.label_6]

        self.label_box_1_layout = QVBoxLayout()
        self.label_box_2_layout = QVBoxLayout()
        self.label_box_3_layout = QVBoxLayout()
        self.label_box_4_layout = QVBoxLayout()
        self.label_box_5_layout = QVBoxLayout()
        self.label_box_6_layout = QVBoxLayout()

        self.label_box_1_layout.addWidget(self.label_1)
        self.label_box_2_layout.addWidget(self.label_2)
        self.label_box_3_layout.addWidget(self.label_3)
        self.label_box_4_layout.addWidget(self.label_4)
        self.label_box_5_layout.addWidget(self.label_5)
        self.label_box_6_layout.addWidget(self.label_6)

        self.label_box_1.setLayout(self.label_box_1_layout)
        self.label_box_2.setLayout(self.label_box_2_layout)
        self.label_box_3.setLayout(self.label_box_3_layout)
        self.label_box_4.setLayout(self.label_box_4_layout)
        self.label_box_5.setLayout(self.label_box_5_layout)
        self.label_box_6.setLayout(self.label_box_6_layout)


        # Ref pos commander setup
        self.line_box_1 = QGroupBox("ref_pos")
        self.line_box_2 = QGroupBox("ref_pos")
        self.line_box_3 = QGroupBox("ref_pos")
        self.line_box_4 = QGroupBox("ref_pos")
        self.line_box_5 = QGroupBox("ref_pos")
        self.line_box_6 = QGroupBox("ref_pos")

        self.line_boxes = [self.line_box_1, self.line_box_2, self.line_box_3, self.line_box_4, self.line_box_5, self.line_box_6]

        self.line_1 = QLineEdit()
        self.line_2 = QLineEdit()
        self.line_3 = QLineEdit()
        self.line_4 = QLineEdit()
        self.line_5 = QLineEdit()
        self.line_6 = QLineEdit()

        self.lines = [self.line_1, self.line_2, self.line_3, self.line_4, self.line_5, self.line_6]

        self.line_1.setMaximumWidth(120)
        self.line_2.setMaximumWidth(120)
        self.line_3.setMaximumWidth(120)
        self.line_4.setMaximumWidth(120)
        self.line_5.setMaximumWidth(120)
        self.line_6.setMaximumWidth(120)

        self.button_1 = QPushButton('set')
        self.button_2 = QPushButton('set')
        self.button_3 = QPushButton('set')
        self.button_4 = QPushButton('set')
        self.button_5 = QPushButton('set')
        self.button_6 = QPushButton('set')

        self.buttons = [self.button_1, self.button_2, self.button_3, self.button_4, self.button_5, self.button_6]

        self.line_box_1_layout = QHBoxLayout()
        self.line_box_2_layout = QHBoxLayout()
        self.line_box_3_layout = QHBoxLayout()
        self.line_box_4_layout = QHBoxLayout()
        self.line_box_5_layout = QHBoxLayout()
        self.line_box_6_layout = QHBoxLayout()

        self.line_box_1_layout.addWidget(self.line_1)
        self.line_box_1_layout.addWidget(self.button_1)
        self.line_box_2_layout.addWidget(self.line_2)
        self.line_box_2_layout.addWidget(self.button_2)
        self.line_box_3_layout.addWidget(self.line_3)
        self.line_box_3_layout.addWidget(self.button_3)
        self.line_box_4_layout.addWidget(self.line_4)
        self.line_box_4_layout.addWidget(self.button_4)
        self.line_box_5_layout.addWidget(self.line_5)
        self.line_box_5_layout.addWidget(self.button_5)
        self.line_box_6_layout.addWidget(self.line_6)
        self.line_box_6_layout.addWidget(self.button_6)

        self.line_box_1.setLayout(self.line_box_1_layout)
        self.line_box_2.setLayout(self.line_box_2_layout)
        self.line_box_3.setLayout(self.line_box_3_layout)
        self.line_box_4.setLayout(self.line_box_4_layout)
        self.line_box_5.setLayout(self.line_box_5_layout)
        self.line_box_6.setLayout(self.line_box_6_layout)


        # One point of control:
        for slider in self.sliders:
            slider.setEnabled(False)
        
        for label in self.labels:
            label.setEnabled(False)
        
        for line in self.lines:
            line.setEnabled(False)
        
        for button in self.buttons:
            button.setEnabled(False)

        self.radio_1 = QRadioButton("GUI control enabled")
        self.radio_1.setChecked(False)

        def radio_state():
            if self.radio_1.isChecked() == True:
                for slider in self.sliders:
                    slider.setEnabled(True)

                for label in self.labels:
                    label.setEnabled(True)

                for line in self.lines:
                    line.setEnabled(True)

                for button in self.buttons:
                    button.setEnabled(True)
            else:
                for slider in self.sliders:
                    slider.setEnabled(False)

                for label in self.labels:
                    label.setEnabled(False)

                for line in self.lines:
                    line.setEnabled(False)

                for button in self.buttons:
                    button.setEnabled(False)

        self.radio_1.toggled.connect(radio_state)


        # inter-widget communications:
        self.slider_1.valueChanged.connect(self.label_1.setNum)
        self.slider_2.valueChanged.connect(self.label_2.setNum)
        self.slider_3.valueChanged.connect(self.label_3.setNum)
        self.slider_4.valueChanged.connect(self.label_4.setNum)
        self.slider_5.valueChanged.connect(self.label_5.setNum)
        self.slider_6.valueChanged.connect(self.label_6.setNum)


        # populate the grid with widgets:
        for slider_box in self.slider_boxes:
            grid.addWidget(slider_box, self.slider_boxes.index(slider_box), 0)
        
        for label_box in self.label_boxes:
            grid.addWidget(label_box, self.label_boxes.index(label_box), 1)

        for line_box in self.line_boxes:
            grid.addWidget(line_box, self.line_boxes.index(line_box), 2)
    
        grid.addWidget(self.radio_1)

        self.setLayout(grid)

        self.setWindowTitle("Meca 500 joint pose controller")
        self.resize(550, 250)


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