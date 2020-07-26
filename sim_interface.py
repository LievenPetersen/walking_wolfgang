"""
this file has been altered, original file:
https://github.com/bit-bots/wolfgang_robot/blob/master/wolfgang_pybullet_sim/src/wolfgang_pybullet_sim/ros_interface.py

Don't look too hard, this is basically the repurposed ROS interface with multiple open wounds.
"""

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
        position, orientation = self.simulation.get_robot_pose()
        linear_vel, angular_vel = self.simulation.get_robot_velocity()
        # simple acceleration computation by using diff of velocities
        linear_acc = tuple(map(lambda i, j: i - j, self.last_linear_vel, linear_vel))
        self.last_linear_vel = linear_vel

        """ # helpful hints for extraction:
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
        return orientation, angular_vel, linear_acc

    def get_foot_pressure(self):
        self.foot_msg_left.left_back = self.simulation.pressure_sensors["LLB"].get_force()
        self.foot_msg_left.left_front = self.simulation.pressure_sensors["LLF"].get_force()
        self.foot_msg_left.right_front = self.simulation.pressure_sensors["LRF"].get_force()
        self.foot_msg_left.right_back = self.simulation.pressure_sensors["LRB"].get_force()

        self.foot_msg_right.left_back = self.simulation.pressure_sensors["RLB"].get_force()
        self.foot_msg_right.left_front = self.simulation.pressure_sensors["RLF"].get_force()
        self.foot_msg_right.right_front = self.simulation.pressure_sensors["RRF"].get_force()
        self.foot_msg_right.right_back = self.simulation.pressure_sensors["RRB"].get_force()

        return self.foot_msg_left, self.foot_msg_right

    def get_true_odom(self):
        return self.simulation.get_robot_pose()

    def joint_goal_cb(self, msg: JointCommand):
        # only put new goals into the goal vector
        i = 0
        for name in msg.joint_names:
            self.simulation.joints[name].set_position(msg.positions[i])
            i += 1

    def reset_sim(self):
        self.simulation.reset()

    """ old stuff I got none to do with and I don't pretend to understand but maybe useful one day...
    def _dynamic_reconfigure_callback(self, config, level):
        self.simulation.set_foot_dynamics(config["contact_damping"], config["contact_stiffness"], config["joint_damping"])
        self.simulation.set_filter_params(config["cutoff"], config["order"])
        return config
    """

    def get_initial_joint_position(self, joint_name):
        return self.simulation.initial_joints_positions.get(joint_name)

    def get_joint(self, name):
        return self.simulation.joints.get(name)
