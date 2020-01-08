import sys
import rclpy
import time
import csv
import os
import math
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import threading
from ament_index_python.packages import get_package_share_directory

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

class CommVariables():
    gui_control_enabled = False
    slider_1_value = 0
    slider_2_value = 0
    slider_3_value = 0
    slider_4_value = 0
    slider_5_value = 0
    slider_6_value = 0
    def __init__(self, parent=None):
        super(CommVariables, self).__init__()

class Window(QWidget, CommVariables):
    def __init__(self, parent=None):
        super(Window, self).__init__(parent)

        self.joint_names = ["meca_axis_1_joint", "meca_axis_2_joint", "meca_axis_3_joint", 
            "meca_axis_4_joint", "meca_axis_5_joint", "meca_axis_6_joint"]

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
        self.label_box_1 = QGroupBox("rad")
        self.label_box_2 = QGroupBox("rad")
        self.label_box_3 = QGroupBox("rad")
        self.label_box_4 = QGroupBox("rad")
        self.label_box_5 = QGroupBox("rad")
        self.label_box_6 = QGroupBox("rad")

        # Indicator labels setup:
        self.label_box_12 = QGroupBox("deg")
        self.label_box_22 = QGroupBox("deg")
        self.label_box_32 = QGroupBox("deg")
        self.label_box_42 = QGroupBox("deg")
        self.label_box_52 = QGroupBox("deg")
        self.label_box_62 = QGroupBox("deg")

        self.label_boxes = [self.label_box_1, self.label_box_2, self.label_box_3, self.label_box_4, self.label_box_5, self.label_box_6]
        self.label_boxes2 = [self.label_box_12, self.label_box_22, self.label_box_32, self.label_box_42, self.label_box_52, self.label_box_62]

        for label_box in self.label_boxes:
            label_box.setMaximumWidth(60)

        for label_box in self.label_boxes2:
            label_box.setMaximumWidth(60)

        self.label_1 = QLabel("value")
        self.label_2 = QLabel("value")
        self.label_3 = QLabel("value")
        self.label_4 = QLabel("value")
        self.label_5 = QLabel("value")
        self.label_6 = QLabel("value")

        self.label_12 = QLabel("value")
        self.label_22 = QLabel("value")
        self.label_32 = QLabel("value")
        self.label_42 = QLabel("value")
        self.label_52 = QLabel("value")
        self.label_62 = QLabel("value")

        self.labels = [self.label_1, self.label_2, self.label_3, self.label_4, self.label_5, self.label_6]
        self.labels2 = [self.label_12, self.label_22, self.label_32, self.label_42, self.label_52, self.label_62]

        self.label_box_1_layout = QVBoxLayout()
        self.label_box_2_layout = QVBoxLayout()
        self.label_box_3_layout = QVBoxLayout()
        self.label_box_4_layout = QVBoxLayout()
        self.label_box_5_layout = QVBoxLayout()
        self.label_box_6_layout = QVBoxLayout()

        self.label_box_12_layout = QVBoxLayout()
        self.label_box_22_layout = QVBoxLayout()
        self.label_box_32_layout = QVBoxLayout()
        self.label_box_42_layout = QVBoxLayout()
        self.label_box_52_layout = QVBoxLayout()
        self.label_box_62_layout = QVBoxLayout()

        self.label_box_1_layout.addWidget(self.label_1)
        self.label_box_2_layout.addWidget(self.label_2)
        self.label_box_3_layout.addWidget(self.label_3)
        self.label_box_4_layout.addWidget(self.label_4)
        self.label_box_5_layout.addWidget(self.label_5)
        self.label_box_6_layout.addWidget(self.label_6)

        self.label_box_12_layout.addWidget(self.label_12)
        self.label_box_22_layout.addWidget(self.label_22)
        self.label_box_32_layout.addWidget(self.label_32)
        self.label_box_42_layout.addWidget(self.label_42)
        self.label_box_52_layout.addWidget(self.label_52)
        self.label_box_62_layout.addWidget(self.label_62)

        self.label_box_1.setLayout(self.label_box_1_layout)
        self.label_box_2.setLayout(self.label_box_2_layout)
        self.label_box_3.setLayout(self.label_box_3_layout)
        self.label_box_4.setLayout(self.label_box_4_layout)
        self.label_box_5.setLayout(self.label_box_5_layout)
        self.label_box_6.setLayout(self.label_box_6_layout)

        self.label_box_12.setLayout(self.label_box_12_layout)
        self.label_box_22.setLayout(self.label_box_22_layout)
        self.label_box_32.setLayout(self.label_box_32_layout)
        self.label_box_42.setLayout(self.label_box_42_layout)
        self.label_box_52.setLayout(self.label_box_52_layout)
        self.label_box_62.setLayout(self.label_box_62_layout)


        # Ref pos commander setup
        self.line_box_1 = QGroupBox("ref_deg")
        self.line_box_2 = QGroupBox("ref_deg")
        self.line_box_3 = QGroupBox("ref_deg")
        self.line_box_4 = QGroupBox("ref_deg")
        self.line_box_5 = QGroupBox("ref_deg")
        self.line_box_6 = QGroupBox("ref_deg")

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

        for label in self.labels2:
            label.setEnabled(False)
        
        for button in self.buttons:
            button.setEnabled(False)

        self.radio_1 = QRadioButton("GUI control enabled")
        self.radio_1.setChecked(False)

        def radio_state():
            if self.radio_1.isChecked() == True:
                CommVariables.gui_control_enabled = True
                for slider in self.sliders:
                    slider.setEnabled(True)

                for label in self.labels:
                    label.setEnabled(True)

                for label in self.labels2:
                    label.setEnabled(True)

                for line in self.lines:
                    line.setEnabled(True)

                for button in self.buttons:
                    button.setEnabled(True)
            else:
                CommVariables.gui_control_enabled = False
                for slider in self.sliders:
                    slider.setEnabled(False)

                for label in self.labels:
                    label.setEnabled(False)

                for label in self.labels2:
                    label.setEnabled(False)

                for line in self.lines:
                    line.setEnabled(False)

                for button in self.buttons:
                    button.setEnabled(False)

        self.radio_1.toggled.connect(radio_state)

        def slider_1_change():
            self.label_1.setText('{0:0.2f}'.format(self.slider_1.value() * math.pi / 180))
            self.label_12.setNum(self.slider_1.value())
            CommVariables.slider_1_value = self.slider_1.value()
        
        def slider_2_change():
            self.label_2.setText('{0:0.2f}'.format(self.slider_2.value() * math.pi / 180))
            self.label_22.setNum(self.slider_2.value())
            CommVariables.slider_2_value = self.slider_2.value()
        
        def slider_3_change():
            self.label_3.setText('{0:0.2f}'.format(self.slider_3.value() * math.pi / 180))
            self.label_32.setNum(self.slider_3.value())
            CommVariables.slider_3_value = self.slider_3.value()
        
        def slider_4_change():
            self.label_4.setText('{0:0.2f}'.format(self.slider_4.value() * math.pi / 180))
            self.label_42.setNum(self.slider_4.value())
            CommVariables.slider_4_value = self.slider_4.value()
        
        def slider_5_change():
            self.label_5.setText('{0:0.2f}'.format(self.slider_5.value() * math.pi / 180))
            self.label_52.setNum(self.slider_5.value())
            CommVariables.slider_5_value = self.slider_5.value()
        
        def slider_6_change():
            self.label_6.setText('{0:0.2f}'.format(self.slider_6.value() * math.pi / 180))
            self.label_62.setNum(self.slider_6.value())
            CommVariables.slider_6_value = self.slider_6.value()

        # inter-widget communications:
        self.slider_1.valueChanged.connect(slider_1_change)
        self.slider_2.valueChanged.connect(slider_2_change)
        self.slider_3.valueChanged.connect(slider_3_change)
        self.slider_4.valueChanged.connect(slider_4_change)
        self.slider_5.valueChanged.connect(slider_5_change)
        self.slider_6.valueChanged.connect(slider_6_change)

        def button_1_clicked():
            self.slider_1.setValue(int(self.line_1.text()))
        
        def button_2_clicked():
            self.slider_2.setValue(int(self.line_2.text()))

        def button_3_clicked():
            self.slider_3.setValue(int(self.line_3.text()))

        def button_4_clicked():
            self.slider_4.setValue(int(self.line_4.text()))

        def button_5_clicked():
            self.slider_5.setValue(int(self.line_5.text()))

        def button_6_clicked():
            self.slider_6.setValue(int(self.line_6.text()))

        self.button_1.clicked.connect(button_1_clicked)
        self.button_2.clicked.connect(button_2_clicked)
        self.button_3.clicked.connect(button_3_clicked)
        self.button_4.clicked.connect(button_4_clicked)
        self.button_5.clicked.connect(button_5_clicked)
        self.button_6.clicked.connect(button_6_clicked)

        # self.slider_1_value = self.slider_1.value()
        self.slider_2_value = self.slider_2.value()
        self.slider_3_value = self.slider_3.value()
        self.slider_4_value = self.slider_4.value()
        self.slider_5_value = self.slider_5.value()
        self.slider_6_value = self.slider_6.value()

        # pose saver widget:
        self.pose_saver_box = QGroupBox("pose_saver")
        self.pose_saver_box_layout = QHBoxLayout()
        self.pose_saver_label = QLabel("pose_name")
        self.pose_saver_line = QLineEdit()
        self.pose_saver_button = QPushButton("update")
        self.pose_saver_box_layout.addWidget(self.pose_saver_label)
        self.pose_saver_box_layout.addWidget(self.pose_saver_line)
        self.pose_saver_box_layout.addWidget(self.pose_saver_button)
        self.pose_saver_box.setLayout(self.pose_saver_box_layout)

        # populate the grid with widgets:
        for slider_box in self.slider_boxes:
            grid.addWidget(slider_box, self.slider_boxes.index(slider_box), 0)
        
        for label_box in self.label_boxes:
            grid.addWidget(label_box, self.label_boxes.index(label_box), 1)

        for label_box in self.label_boxes2:
            grid.addWidget(label_box, self.label_boxes2.index(label_box), 2)

        for line_box in self.line_boxes:
            grid.addWidget(line_box, self.line_boxes.index(line_box), 3)
    
        grid.addWidget(self.pose_saver_box, 6, 0, 1, 4)
        grid.addWidget(self.radio_1, 7, 0)


        self.setLayout(grid)

        self.setWindowTitle("Meca 500 joint pose controller")
        self.resize(550, 250)


class Meca500R3Sim(Node, CommVariables):

    def __init__(self):
        super().__init__("ros2_mecademic_sim")

        # self.cv = CommVariables()
        # self.active_button = False

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
        if CommVariables.gui_control_enabled == False:
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
        else:
        # print(CommVariables.slider_1_value)
            go_to_pos = [round(CommVariables.slider_1_value * math.pi / 180, 2),
                round(CommVariables.slider_2_value * math.pi / 180, 2),
                round(CommVariables.slider_3_value * math.pi / 180, 2),
                round(CommVariables.slider_4_value * math.pi / 180, 2),
                round(CommVariables.slider_5_value * math.pi / 180, 2),
                round(CommVariables.slider_6_value * math.pi / 180, 2)]

            for i in range(0, 6):
                if go_to_pos != None:
                    if go_to_pos[i] < self.act_pos[i]:
                        self.pub_pos[i] = round(self.act_pos[i] - 0.01, 2)
                    elif go_to_pos[i] > self.act_pos[i]:
                        self.pub_pos[i] = round(self.act_pos[i] + 0.01, 2)
                    else:
                        pass

            self.ref_pos = go_to_pos

        self.to_r.name = self.joint_names
        self.to_r.position = self.pub_pos
        self.joint_cmd_publisher_.publish(self.to_r)
        
    def interfacer_to_sp_publisher_callback(self):
        print(CommVariables.gui_control_enabled)
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

    # print(ros2_mecademic_sim.active_button) # = clock.active_button
    
if __name__ == '__main__':
    main()