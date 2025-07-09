"""receives joint positions from the high-level controller and sends corresponding torque commands according to PD control"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.wait_for_message import wait_for_message
from std_msgs.msg import Float64


command_topics = ["left_hip_yaw_joint_cmd", "left_hip_pitch_joint_cmd", "left_hip_roll_joint_cmd",
                  "left_knee_joint_cmd", "left_ankle_pitch_joint_cmd", "left_ankle_roll_joint_cmd",
                  "right_hip_yaw_joint_cmd", "right_hip_pitch_joint_cmd", "right_hip_roll_joint_cmd",
                  "right_knee_joint_cmd", "right_ankle_pitch_joint_cmd", "right_ankle_roll_joint_cmd",
                  "torso_joint_cmd", "left_shoulder_pitch_joint_cmd", "left_shoulder_roll_joint_cmd",
                  "left_shoulder_yaw_joint_cmd", "left_elbow_joint_cmd", "left_wrist_roll_joint_cmd",
                  "left_wrist_pitch_joint_cmd", "left_wrist_yaw_joint_cmd", "right_shoulder_pitch_joint_cmd",
                  "right_shoulder_roll_joint_cmd", "right_shoulder_yaw_joint_cmd", "right_elbow_joint_cmd", 
                  "right_wrist_roll_joint_cmd", "right_wrist_pitch_joint_cmd", "right_wrist_yaw_joint_cmd"]

class LLController(Node):
    def __init__(self):
        super().__init__('a1_controller')
        self.freq = 500
        self.timer = self.create_timer(1.0 / self.freq, self.update)
    
        self.create_subscription(JointState, "/goal/joint_states", self.joint_cmd_callback, 10)
        self.create_subscription(JointState, "/joint_states", self.joint_states_callback, 10)

        _,msg = wait_for_message(msg_type=JointState, node=self,topic="/joint_states",time_to_wait=5)
        self.velocities = msg.velocity
        self.positions = msg.position

        self.pubs = []
        for topic in command_topics:
            self.pubs.append(self.create_publisher(Float64, topic, 10))

        self.desired_thetas = self.positions
        self.cmd_thetas = [0.0]*len(self.pubs)

        self.kp = [175.0]*len(self.pubs)
        self.kp[14:] = [100.0] * (len(self.kp) - 14)

        self.kd = [0.2]*len(self.pubs)



    """Main loop of the controller, updates at self.freq"""
    def update(self):

        for i in range(len(self.pubs)):
            # PD Controller
            self.cmd_thetas[i] = self.kp[i] * (self.desired_thetas[i] - self.positions[i]) - self.kd[i] * self.velocities[i]

            motor_cmd = Float64()
            motor_cmd.data = self.cmd_thetas[i]
            self.pubs[i].publish(motor_cmd)

    def joint_cmd_callback(self, msg):
        self.desired_thetas = msg.position
        
    def joint_states_callback(self, msg):
        self.positions = msg.position
        self.velocities = msg.velocity
    

def main(args=None):
    rclpy.init(args=args)
    controller = LLController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


