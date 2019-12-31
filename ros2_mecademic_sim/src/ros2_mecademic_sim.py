import sys
import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState

class Meca500R3Sim(Node):

    def __init__(self):
        super().__init__("ros2_mecademic_sim")

        self.from_sp = String()
        self.to_sp = String()
        self.from_r = JointState()
        self.to_r = JointState()
        self.tmr_period = 0.02
        self.tmr_period2 = 0.5

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

        if self.act_pos == [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]:
            self.act_pos_str = "away"
        elif self.act_pos == [1.0, 1.0, 1.0, 1.0, 1.0 ,1.0]:
            self.act_pos_str = "at"
        else:
            pass

    def sp_callback(self, data):
        self.ref_pos_str = data.data
        if self.ref_pos_str == "away":
            self.ref_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        elif self.ref_pos_str == "at":
            self.ref_pos = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        else:
            pass

    def interfacer_to_r_publisher_callback(self):
        for i in range(0, 6):
            if self.ref_pos[i] < self.act_pos[i]:
                self.pub_pos[i] = round(self.act_pos[i] - 0.01, 2)
            elif self.ref_pos[i] > self.act_pos[i]:
                self.pub_pos[i] = round(self.act_pos[i] + 0.01, 2)
            else:
                pass
        self.to_r.name = ["meca_axis_1_joint", "meca_axis_2_joint", "meca_axis_3_joint", "meca_axis_4_joint", "meca_axis_5_joint", "meca_axis_6_joint"]
        self.to_r.position = self.pub_pos
        self.joint_cmd_publisher_.publish(self.to_r)
        
    def interfacer_to_sp_publisher_callback(self):
        if self.act_pos_str == "at":
            self.to_sp.data = "at"
        else:
            self.to_sp.data = "unknown"

        self.state_to_sp_publisher_.publish(self.to_sp)

def main(args=None):
    rclpy.init(args=args)

    ros2_mecademic_sim = Meca500R3Sim()

    rclpy.spin(ros2_mecademic_sim)

    ros2_mecademic_sim.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()