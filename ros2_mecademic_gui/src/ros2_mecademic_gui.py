import sys
import time
import csv
import os
import math
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from ros2_mecademic_msgs.msg import MecademicGuiToEsd
from ros2_mecademic_msgs.msg import MecademicEsdToGui
from ros2_mecademic_msgs.msg import MecademicGuiToUtils
from ros2_mecademic_msgs.msg import MecademicUtilsToGui
from ament_index_python.packages import get_package_share_directory

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

class CommVariables():
    gui_control_enabled = False
    speed_slider_value = 0
    slider_1_value = 0
    slider_2_value = 0
    slider_3_value = 0
    slider_4_value = 0
    slider_5_value = 0
    slider_6_value = 0
    pose_name = ""
    actual_pose = ""
    actual_joint_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    saved_poses = ""
    def __init__(self, parent=None):
        super(CommVariables, self).__init__()

# class QDoubleSlider(QSlider):

#     # create our our signal that we can connect to if necessary
#     doubleValueChanged = pyqtSignal(float)

#     def __init__(self, decimals = 5, *args, **kargs):
#         super(QDoubleSlider, self).__init__( *args, **kargs)
#         self._multi = 10 ** decimals

#         self.valueChanged.connect(self.emitDoubleValueChanged)

#     def emitDoubleValueChanged(self):
#         value = float(super(QDoubleSlider, self).value())/self._multi
#         self.doubleValueChanged.emit(value)

#     def value(self):
#         return float(super(QDoubleSlider, self).value()) / self._multi

#     def setMinimum(self, value):
#         return super(QDoubleSlider, self).setMinimum(value * self._multi)

#     def setMaximum(self, value):
#         return super(QDoubleSlider, self).setMaximum(value * self._multi)

#     def setSingleStep(self, value):
#         return super(QDoubleSlider, self).setSingleStep(value * self._multi)

#     def singleStep(self):
#         return float(super(QDoubleSlider, self).singleStep()) / self._multi

#     def setValue(self, value):
#         super(QDoubleSlider, self).setValue(int(value * self._multi))

class Window(QWidget, CommVariables):
    def __init__(self, parent=None):
        super(Window, self).__init__(parent)

        self.joint_names = ["meca_axis_1_joint", "meca_axis_2_joint", "meca_axis_3_joint", 
            "meca_axis_4_joint", "meca_axis_5_joint", "meca_axis_6_joint"]

        minimum = -180
        maximum = 180
        step = 0.0001

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

                self.speed_box.setEnabled(True)
                self.pose_saver_box.setEnabled(True)

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

                self.speed_box.setEnabled(False)
                self.pose_saver_box.setEnabled(False)

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

        # speedwidget:
        self.speed_box = QGroupBox("robot_speed")
        self.speed_box_layout = QHBoxLayout()
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setFocusPolicy(Qt.StrongFocus)
        self.speed_slider.setTickPosition(QSlider.TicksBothSides)
        self.speed_slider.setMinimum(0)
        self.speed_slider.setMaximum(100)
        self.speed_slider.setSingleStep(step)
        self.speed_slider.setMinimumWidth(300)
        self.speed_line = QLineEdit("%")
        self.speed_line.setMaximumWidth(55)
        self.speed_button = QPushButton("set")
        self.speed_box_layout.addWidget(self.speed_slider)
        self.speed_box_layout.addWidget(self.speed_line)
        self.speed_box_layout.addWidget(self.speed_button)
        self.speed_box.setLayout(self.speed_box_layout)
        self.speed_box.setEnabled(False)

        def speed_slider_change():
            CommVariables.speed_slider_value = self.speed_slider.value()

        def speed_button_clicked():
            self.speed_slider.setValue(int(self.speed_line.text()))

        self.speed_button.clicked.connect(speed_button_clicked)
        self.speed_slider_value = self.speed_slider.value()
        self.speed_slider.valueChanged.connect(speed_slider_change)

        # pose saver widget:
        self.pose_saver_box = QGroupBox("pose_saver")
        self.pose_saver_box_layout = QHBoxLayout()
        self.pose_saver_label = QLabel("pose_name")
        self.pose_saver_line = QLineEdit("some_pose_name")
        self.pose_saver_button = QPushButton("update")
        self.pose_saver_box_layout.addWidget(self.pose_saver_label)
        self.pose_saver_box_layout.addWidget(self.pose_saver_line)
        self.pose_saver_box_layout.addWidget(self.pose_saver_button)
        self.pose_saver_box.setLayout(self.pose_saver_box_layout)
        self.pose_saver_box.setEnabled(False)

        def pose_saver_button_clicked():
            CommVariables.pose_name = self.pose_saver_line.text()

        self.pose_saver_button.clicked.connect(pose_saver_button_clicked)

        # populate the grid with widgets:
        for slider_box in self.slider_boxes:
            grid.addWidget(slider_box, self.slider_boxes.index(slider_box), 0)
        
        for label_box in self.label_boxes:
            grid.addWidget(label_box, self.label_boxes.index(label_box), 1)

        for label_box in self.label_boxes2:
            grid.addWidget(label_box, self.label_boxes2.index(label_box), 2)

        for line_box in self.line_boxes:
            grid.addWidget(line_box, self.line_boxes.index(line_box), 3)
    
        grid.addWidget(self.speed_box, 6, 0, 1, 4)
        grid.addWidget(self.pose_saver_box, 7, 0, 1, 4)
        grid.addWidget(self.radio_1, 8, 0)

        self.setLayout(grid)

        self.setWindowTitle("Meca 500 joint pose controller")
        self.resize(550, 250)

class Ros2MecademicGui(Node, CommVariables):

    def __init__(self):
        super().__init__("ros2_mecademic_gui")

        self.gui_to_esd_msg = MecademicGuiToEsd()
        self.gui_to_utils_msg = MecademicGuiToUtils()
        self.joint_state = JointState()

        self.gui_to_esd_msg.gui_control_enabled = False
        self.gui_to_esd_msg.gui_speed_control = 0
        self.gui_to_esd_msg.gui_joint_control = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.actual_pose = ""
        self.actual_joint_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.saved_poses = []

        self.gui_to_esd_timer_period = 0.02
        self.gui_to_utils_timer_period = 0.2

        self.joint_names = ["meca_axis_1_joint", "meca_axis_2_joint", "meca_axis_3_joint", 
            "meca_axis_4_joint", "meca_axis_5_joint", "meca_axis_6_joint"]

        self.utils_to_gui_subscriber = self.create_subscription(
            MecademicUtilsToGui, 
            "/mecademic_utils_to_gui",
            self.utils_to_gui_callback,
            10)

        self.esd_to_gui_subscriber = self.create_subscription(
            MecademicEsdToGui, 
            "/mecademic_esd_to_gui",
            self.esd_to_gui_callback,
            10)

        self.joint_state_subscriber = self.create_subscription(
            JointState, 
            "/mecademic_joint_state",
            self.joint_state_callback,
            10)

        time.sleep(2)

        self.gui_to_esd_publisher_ = self.create_publisher(
            MecademicGuiToEsd,
            "/mecademic_gui_to_esd",
            10)

        self.gui_to_utils_publisher_ = self.create_publisher(
            MecademicGuiToUtils,
            "/mecademic_gui_to_utils",
            10)
        
        self.gui_to_esd_timer = self.create_timer(
            self.gui_to_esd_timer_period, 
            self.gui_to_esd_callback)

        self.gui_to_utils_timer = self.create_timer(
            self.gui_to_utils_timer_period, 
            self.gui_to_utils_callback)

    def gui_to_esd_callback(self):
        self.gui_to_esd_msg.gui_control_enabled = CommVariables.gui_control_enabled
        
        if CommVariables.speed_slider_value >= 0 and CommVariables.speed_slider_value <= 100:
            self.gui_to_esd_msg.gui_speed_control = CommVariables.speed_slider_value
        else:
            pass

        self.gui_to_esd_msg.gui_joint_control[0] = CommVariables.slider_1_value * math.pi / 180
        self.gui_to_esd_msg.gui_joint_control[1] = CommVariables.slider_2_value * math.pi / 180
        self.gui_to_esd_msg.gui_joint_control[2] = CommVariables.slider_3_value * math.pi / 180
        self.gui_to_esd_msg.gui_joint_control[3] = CommVariables.slider_4_value * math.pi / 180
        self.gui_to_esd_msg.gui_joint_control[4] = CommVariables.slider_5_value * math.pi / 180
        self.gui_to_esd_msg.gui_joint_control[5] = CommVariables.slider_6_value * math.pi / 180

        self.gui_to_esd_publisher_.publish(self.gui_to_esd_msg)

    def gui_to_utils_callback(self):
        self.gui_to_utils_msg.utility_action = "update"
        self.gui_to_utils_msg.utility_pose_name = CommVariables.pose_name
        self.gui_to_utils_publisher_.publish(self.gui_to_utils_msg)

    def esd_to_gui_callback(self, data):
        self.actual_pose = data.actual_pose
        CommVariables.actual_pose = self.actual_pose
        
    def utils_to_gui_callback(self, data):
        self.saved_poses = data.saved_poses
        CommVariables.saved_poses = self.saved_poses

    def joint_state_callback(self, data):
        self.actual_joint_pose = data.position
        CommVariables.actual_joint_pose = self.actual_joint_pose

def main(args=None):

    def launch_node():
        def launch_node_callback_local():
            rclpy.init(args=args)
            ros2_mecademic_gui = Ros2MecademicGui()
            rclpy.spin(ros2_mecademic_gui)
            ros2_mecademic_gui.destroy_node()
            rclpy.shutdown()
        t = threading.Thread(target=launch_node_callback_local)
        t.daemon = True
        t.start()
    
    # Window has to be in the main thread
    def launch_window():
        app = QApplication(sys.argv)
        clock = Window()
        clock.show()
        sys.exit(app.exec_())

    launch_node()    
    launch_window()

if __name__ == '__main__':
    main()