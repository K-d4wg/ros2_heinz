import numpy as np
import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from rclpy.wait_for_message import wait_for_message

joint_cmd_topics = [
    "/h1/left_hip_yaw_joint/cmd_pos",
    "/h1/left_hip_pitch_joint/cmd_pos",
    "/h1/left_hip_roll_joint/cmd_pos",
    "/h1/left_knee_joint/cmd_pos",
    "/h1/left_ankle_pitch_joint/cmd_pos",
    "/h1/left_ankle_roll_joint/cmd_pos",
    "/h1/right_hip_yaw_joint/cmd_pos",
    "/h1/right_hip_pitch_joint/cmd_pos",
    "/h1/right_hip_roll_joint/cmd_pos",
    "/h1/right_knee_joint/cmd_pos",
    "/h1/right_ankle_pitch_joint/cmd_pos",
    "/h1/right_ankle_roll_joint/cmd_pos",
    "/h1/torso_joint/cmd_pos",
    "/h1/left_shoulder_pitch_joint/cmd_pos",
    "/h1/left_shoulder_roll_joint/cmd_pos",
    "/h1/left_shoulder_yaw_joint/cmd_pos",
    "/h1/left_elbow_joint/cmd_pos",
    "/h1/left_wrist_roll_joint/cmd_pos",
    "/h1/left_wrist_pitch_joint/cmd_pos",
    "/h1/left_wrist_yaw_joint/cmd_pos",
    "/h1/right_shoulder_pitch_joint/cmd_pos",
    "/h1/right_shoulder_roll_joint/cmd_pos",
    "/h1/right_shoulder_yaw_joint/cmd_pos",
    "/h1/right_elbow_joint/cmd_pos",
    "/h1/right_wrist_roll_joint/cmd_pos",
    "/h1/right_wrist_pitch_joint/cmd_pos",
    "/h1/right_wrist_yaw_joint/cmd_pos"
]


class example_controller(Node):
    def __init__(self):
        super().__init__('example_controller')
        self.frequency = 10
        self.timer = self.create_timer(1.0 / self.frequency, self.update)
        self.create_subscription(JointState, "/joint_states", self.joint_states_callback, 10)
        
        _, self.first_pos = wait_for_message(msg_type=JointState, node=self,topic="/joint_states",time_to_wait=5)
        print("First joint state received:", self.first_pos)
        self.startup_pos = np.array(self.first_pos.position)

        self.goal_pos = np.zeros(len(joint_cmd_topics))
        self.goal_pos[20] = -1.76  # right shoulder pitch
        self.goal_pos[21] = -1.34  # right shoulder roll

        self.pub = []
        for topic in joint_cmd_topics:
            self.pub.append(self.create_publisher(Float64, topic, 10))

        self.motor_command = Float64()

        self.init_pose(3)
        self.timestamp = 0


    def init_pose(self, t):
        print("Moving into default position...")
        
        num_steps = self.frequency*t
        step = (self.goal_pos - self.startup_pos) / num_steps
        for _ in range(num_steps):
            self.startup_pos[20] += step[20]
            self.motor_command.data = self.startup_pos[20]
            self.pub[20].publish(self.motor_command)

            self.startup_pos[21] += step[21]
            self.motor_command.data = self.startup_pos[21]
            self.pub[21].publish(self.motor_command)
            time.sleep(1.0/self.frequency)
        print("Default position reached.")


    def update(self):
        # here is the main control loop

        self.motor_command.data = np.sin(self.timestamp/self.frequency)
        self.pub[23].publish(self.motor_command)  # elbow joint
        self.timestamp += 1
        pass

    def joint_states_callback(self, msg):
        self.positions = msg.position

def main(args=None):
    rclpy.init(args=args)
    print("example")

    controller = example_controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()