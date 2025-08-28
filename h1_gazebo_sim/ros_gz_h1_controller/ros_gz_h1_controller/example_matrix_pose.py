import numpy as np
import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from rclpy.wait_for_message import wait_for_message

joint_cmd_topics = [
    "/h1/left_hip_yaw_joint/cmd_pos",   
    "/h1/left_hip_pitch_joint/cmd_pos",   # 1
    "/h1/left_hip_roll_joint/cmd_pos",
    "/h1/left_knee_joint/cmd_pos",  # 3
    "/h1/left_ankle_pitch_joint/cmd_pos",   # 4
    "/h1/left_ankle_roll_joint/cmd_pos",   
    "/h1/right_hip_yaw_joint/cmd_pos",
    "/h1/right_hip_pitch_joint/cmd_pos",   # 7
    "/h1/right_hip_roll_joint/cmd_pos",
    "/h1/right_knee_joint/cmd_pos",   # 9
    "/h1/right_ankle_pitch_joint/cmd_pos",   # 10
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
        self.frequency = 100
        self.timer = self.create_timer(1.0 / self.frequency, self.update)
        self.create_subscription(JointState, "/joint_states", self.joint_states_callback, 10)
        
        _, self.first_pos = wait_for_message(msg_type=JointState, node=self,topic="/joint_states",time_to_wait=5)
        print("First joint state received:", self.first_pos)
        self.startup_pos = np.array(self.first_pos.position)

        self.goal_pos = np.zeros(len(joint_cmd_topics))
        self.goal_pos[1] = 1.8
        self.goal_pos[3] = 1.16915
        self.goal_pos[4] = -0.892202
        self.goal_pos[7] = 1.8
        self.goal_pos[9] = 1.16915
        self.goal_pos[10] = -0.892202

        self.pub = []
        for topic in joint_cmd_topics:
            self.pub.append(self.create_publisher(Float64, topic, 10))

        self.motor_command = Float64()

        self.init_pose(1)
        self.timestamp = 0


    def init_pose(self, t):
        print("Moving into default position...")
        print(self.goal_pos)
        num_steps = self.frequency*t
        step = (self.goal_pos - self.startup_pos) / num_steps
        for _ in range(num_steps):
            for idx in [1,7]:
                self.startup_pos[idx] += step[idx]
                self.motor_command.data = self.startup_pos[idx]
                self.pub[idx].publish(self.motor_command)

                # knee control
                self.startup_pos[idx+2] += step[idx+2]
                self.motor_command.data = self.startup_pos[idx+2]
                self.pub[idx+2].publish(self.motor_command)

                # ankle control
                self.startup_pos[idx+3] += step[idx+3]
                self.motor_command.data = self.startup_pos[idx+3]
                self.pub[idx+3].publish(self.motor_command)
            time.sleep(1.0/self.frequency)
        print("Default position reached.")


    def update(self):
        # here is the main control loop
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