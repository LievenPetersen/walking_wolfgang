"""
this file has been altered, original file:
https://github.com/bit-bots/wolfgang_robot/blob/master/wolfgang_pybullet_sim/src/wolfgang_pybullet_sim/ros_interface.py
"""

import time
# import rospy
# from bitbots_msgs.msg import FootPressure, JointCommand
# from nav_msgs.msg import Odometry
# from rosgraph_msgs.msg import Clock
# from sensor_msgs.msg import JointState, Imu
# from std_msgs.msg import Float32, Bool
from simulation import Simulation


class Msg:
    def __init__(self):
        self.frame_id = ""


class JointState(Msg):
    def __init__(self):
        super().__init__()
        self.header = 0
        self.name = []


class FootPressure(Msg):
    def __init__(self):
        super().__init__()
        self.left_back = 0
        self.left_front = 0
        self.right_front = 0
        self.right_back = 0


class Odometry(Msg):
    def __init__(self):
        super().__init__()
        self.child_frame_id = ""
        self.position = []
        self.orientation = []


class JointCommand:
    def __init__(self):
        self.positions = []
        self.joint_names = []


class SimInterface:
    def __init__(self, simulation: Simulation):

        self.simulation = simulation
        self.last_linear_vel = (0, 0, 0)

        # messages
        self.joint_state_msg = JointState()
        self.joint_state_msg.frame_id = "base_link"
        self.joint_state_msg.name = self.simulation.initial_joints_positions.keys()
        # self.imu_msg = Imu()
        # self.imu_msg.header.frame_id = "imu_frame"
        self.foot_msg_left = FootPressure()
        self.foot_msg_left.frame_id = 'l_sole'
        self.foot_msg_right = FootPressure()
        self.foot_msg_right.frame_id = 'r_sole'
        self.odom_msg = Odometry()
        self.odom_msg.frame_id = "odom"
        self.odom_msg.child_frame_id = "base_link"

    def publish_joints(self):
        positions = []
        velocities = []
        efforts = []
        for name in self.joint_state_msg.name:
            joint = self.simulation.joints[name]
            positions.append(joint.get_position())
            velocities.append(joint.get_velocity())
            efforts.append(joint.get_torque())
        self.joint_state_msg.position = positions
        self.joint_state_msg.velocity = velocities
        self.joint_state_msg.effort = efforts
        # self.joint_publisher.publish(self.joint_state_msg)

    def publish_imu(self):
        """
        self.imu_msg.orientation.x = orientation[0]
        self.imu_msg.orientation.y = orientation[1]
        self.imu_msg.orientation.z = orientation[2]
        self.imu_msg.orientation.w = orientation[3]
        self.imu_msg.angular_velocity.x = angular_vel[0]
        self.imu_msg.angular_velocity.y = angular_vel[1]
        self.imu_msg.angular_velocity.z = angular_vel[2]
        self.imu_msg.linear_acceleration.x = linear_acc[0]
        self.imu_msg.linear_acceleration.y = linear_acc[0]
        self.imu_msg.linear_acceleration.z = linear_acc[0]
        """

        position, orientation = self.simulation.get_robot_pose()
        linear_vel, angular_vel = self.simulation.get_robot_velocity()
        # simple acceleration computation by using diff of velocities
        linear_acc = tuple(map(lambda i, j: i - j, self.last_linear_vel, linear_vel))
        self.last_linear_vel = linear_vel

        return orientation, angular_vel, linear_acc

    def get_foot_pressure(self):
        self.calculate_foot_pressure(False)
        return self.foot_msg_left, self.foot_msg_right

    def get_filtered_foot_pressure(self):
        self.calculate_foot_pressure(True)
        return self.foot_msg_left, self.foot_msg_right

    def calculate_foot_pressure(self, filtered: bool):
        f_llb = self.simulation.pressure_sensors["LLB"].get_force()
        f_llf = self.simulation.pressure_sensors["LLF"].get_force()
        f_lrf = self.simulation.pressure_sensors["LRF"].get_force()
        f_lrb = self.simulation.pressure_sensors["LRB"].get_force()

        f_rlb = self.simulation.pressure_sensors["RLB"].get_force()
        f_rlf = self.simulation.pressure_sensors["RLF"].get_force()
        f_rrf = self.simulation.pressure_sensors["RRF"].get_force()
        f_rrb = self.simulation.pressure_sensors["RRB"].get_force()

        self.foot_msg_left.left_back = f_llb
        self.foot_msg_left.left_front = f_llf
        self.foot_msg_left.right_front = f_lrf
        self.foot_msg_left.right_back = f_lrb

        self.foot_msg_right.left_back = f_rlb
        self.foot_msg_right.left_front = f_rlf
        self.foot_msg_right.right_front = f_rrf
        self.foot_msg_right.right_back = f_rrb

    def get_true_odom(self):
        return self.simulation.get_robot_pose()

    def joint_goal_cb(self, msg: JointCommand):
        # only put new goals into the goal vector
        i = 0
        for name in msg.joint_names:
            self.simulation.joints[name].set_position(msg.positions[i])
            i += 1

    def reset_cb(self, msg):
        self.simulation.reset()

    def _dynamic_reconfigure_callback(self, config, level):
        self.simulation.set_foot_dynamics(config["contact_damping"], config["contact_stiffness"], config["joint_damping"])
        self.simulation.set_filter_params(config["cutoff"], config["order"])
        return config