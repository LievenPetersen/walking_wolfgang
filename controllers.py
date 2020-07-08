import math
import abc

import pybullet as p

from sim_interface import SimInterface


class IController:
    def __init__(self, initial_point):
        self.integral = 0
        self.last_actual_point = initial_point
        self.last_desired_point = initial_point

    def input(self, desired_point, actual_point):
        self.integral += desired_point - self.last_desired_point - (actual_point - self.last_actual_point)
        self.last_actual_point = actual_point

    def output(self):
        return self.integral


class Controller:
    @abc.abstractmethod
    def update(self):
        pass


class SmoothMotorController(Controller):
    """provides abstract control for a given joint"""

    def __init__(self, name, interface: SimInterface):
        super().__init__()
        assert interface.get_joint(name) is not None  # joint exists (i.e. did you spell it right?)
        self.interface = interface

        self.DEBUG = False

        self.name = name  # name of the joint
        self.joint = interface.get_joint(name)  # joint object
        self.initial_position = math.radians(interface.get_initial_joint_position(name))

        self.i_controller = IController(self.initial_position)

        self.moving = False  # true while executing a movement
        self.current_start_position = self.initial_position  # were the current move started from
        self.current_target_position = self.initial_position  # were the current move should end
        # (unless manually overridden this is the current position outside of movements)
        self.current_start_time = 0  # simulation time of the start of the movement
        self.current_target_time = 0  # at which simulation time the target should be reached

        self.t1 = 0
        self.t2 = 0
        self.a1 = 0
        self.v2 = 0

        if self.name[0] is "L":
            self.side = -1
        else:
            self.side = 1

    def update(self):
        # movement is done
        if self.moving and self.current_target_time <= self.interface.simulation.time:
            self.moving = False
            self.current_start_position = self.current_target_position
            self.joint.set_position(self.current_target_position)
            if self.name is "RKnee":
                print("move ended", self.current_target_position - self.joint.get_position())

        elif self.moving:
            self.joint.set_position(self.get_current_position())

    def goto_initial_position(self):
        self.joint.set_position(math.radians(self.initial_position))

    def reach_position_in_time(self, position, time_to_reach):  # position in radians, time_to_reach in seconds
        self.current_start_position = self.get_current_position()
        self.current_target_position = position
        delta_position = self.current_target_position - self.current_start_position

        self.current_start_time = self.interface.simulation.time
        self.current_target_time = self.current_start_time + time_to_reach

        self.moving = True
        """ 
        smooth movements happen in 3 phases:
            1. Phase: constant acceleration up to the coasting speed
            2. Phase: constant speed for the duration of coasting_ratio * time_to_reach
            3. Phase: constant -acceleration until standstill
        The relative time of each phase is always the same ratio, defined by coasting_ratio.
        """
        coasting_ratio = 0.5  # percentage of non accelerated time in between acceleration phases of a movement
        self.t1 = time_to_reach * (1 - coasting_ratio) / 2  # time of phase 1 or phase 3
        self.t2 = time_to_reach * coasting_ratio  # time of phase 2
        self.v2 = delta_position / self.t2  # speed during phase 2
        self.a1 = self.v2 / self.t1  # acceleration during phase 1 or -acceleration of phase 3

    def get_moving_position(self):
        tc = self.get_passed_time_of_current_movement()  # ie current time
        assert not tc < 0  # invalid current time since start of the movement

        result = 0
        # first phase
        if tc <= self.t1:
            result = self.a1 * tc**2 * 0.5

        # second phase
        elif tc <= self.t1 + self.t2:
            t = tc - self.t1
            result = (self.a1 * self.t1**2 * 0.5) + (self.v2 * t)

        # third phase
        elif tc <= 2 * self.t1 + self.t2:
            t = tc - (self.t1 + self.t2)
            result = (self.a1 * self.t1**2 * 0.5) + (self.v2 * self.t2) + (self.v2 * t - self.a1 * t**2 * 0.5)

        return result * 2/3  # dirty fix for my problems (these calculations are 150% of the speed)
        # breaks again if coasting ratio is changed

    def get_name(self):
        return self.name

    def get_current_position(self):
        if not self.moving:
            return self.current_target_position
        return self.current_start_position + self.get_moving_position()

    def get_passed_time_of_current_movement(self):
        return self.interface.simulation.time - self.current_start_time


class MotorController(Controller):
    """provides abstract control for a given joint"""

    offsets = {"LHipPitch": -1.5708 + math.pi/2, "LKnee": -0.261799 - math.pi, "LAnklePitch": 2.52888e-15,
               "RHipPitch": (1.5708 - math.pi/2), "RKnee": 0.261799 + math.pi, "RAnklePitch": -2.52888e-15}

    def __init__(self, name, interface: SimInterface):
        super().__init__()
        assert interface.get_joint(name) is not None  # joint exists (i.e. did you spell it right?)
        self.interface = interface

        self.DEBUG = False

        self.name = name  # name of the joint
        self.joint = interface.get_joint(name)  # joint object
        self.initial_position = math.radians(interface.get_initial_joint_position(name))

        self.radiant_error_tolerance = 0.001  # if position-error is greater than error_tolerance vel. is recalculated

        self.current_start_position = 0  # were the current move started from
        self.current_target_position = self.initial_position  # were the current move should end
        # (unless manually overridden this is the current position outside of movements)
        self.current_speed = 0  # how fast the current move should be
        self.current_start_time = 0  # simulation time of the start of the movement
        self.current_target_time = 0  # at which simulation time the target should be reached

        if self.name[0] is "L":
            self.side = -1
        else:
            self.side = 1

    def update(self):
        if self.current_target_time is not 0 and self.current_target_time <= self.interface.simulation.time:
            self.joint.set_position(self.current_target_position)
            self.current_speed = 0
            self.current_target_time = 0

            if self.DEBUG:
                print("reached", self.get_name(), "position with error",
                      self.joint.get_position() - self.current_target_position)

        # if the positional error is greater than the error tolerance:
        elif self.current_target_time is not 0 \
                and abs(self.joint.get_position() - self.get_predicted_position()) > self.radiant_error_tolerance:

            if self.DEBUG:
                print("correcting", self.get_name(), "error:",
                      float(self.joint.get_position()) - self.get_predicted_position())
            # calculate a new velocity to reach the position in time
            self.reach_position_in_time(self.current_target_position,
                                        self.current_target_time - self.interface.simulation.time)

    def goto_initial_position(self):
        self.joint.set_position(math.radians(self.initial_position))

    def reach_position_in_time(self, position, time_to_reach):  # position in radians, time_to_reach in seconds
        self.current_start_position = self.joint.get_position()
        self.current_start_time = self.interface.simulation.time
        self.current_target_position = position
        delta_position = self.current_target_position - self.current_start_position

        if time_to_reach > 0:
            self.current_speed = delta_position / time_to_reach
        else:
            self.current_speed = self.joint.max_velocity

        self.joint.set_velocity(velocity=self.current_speed)
        self.current_target_time = self.interface.simulation.time + time_to_reach

    def reach_position_in_time_smooth(self, position, time_to_reach):
        self.current_start_position = self.joint.get_position()
        self.current_start_time = self.interface.simulation.time
        self.current_target_position = position
        delta_position = self.current_target_position - self.current_start_position

        if time_to_reach > 0:
            self.current_speed = delta_position / time_to_reach
        else:
            self.current_speed = self.joint.max_velocity

        self.joint.set_velocity(velocity=self.current_speed)
        self.current_target_time = self.interface.simulation.time + time_to_reach

    def get_name(self):
        return self.name

    def get_predicted_position(self):
        return self.current_start_position + self.get_duration_of_current_movement() * self.current_speed

    def get_duration_of_current_movement(self):
        return self.interface.simulation.time - self.current_start_time


class Leg(Controller):
    def __init__(self, hip: MotorController, knee: MotorController, ankle: MotorController, side: int,
                 interface: SimInterface):
        """side: left = -1, right = 1"""
        super().__init__()
        self.robot_id = interface.simulation.get_robot_id()
        self.side = side
        self.hip = hip
        self.knee = knee
        self.ankle = ankle

        # offsets are the angular difference between motor orientation and 90° or 180° angles (copied from URDF file)
        self.hip_offset = (1.5708 - math.pi/2) * self.side
        self.knee_offset = -0.261799 * self.side
        self.ankle_offset = -2.52888e-15 * self.side

        self.hip_knee_length = 0.17661235909670223  # calculated from URDF file (probably correct)
        self.knee_ankle_length = 0.1773421833631243  # calculated from URDF file (probably correct)
        self.length_upper_limit = self.hip_knee_length + self.knee_ankle_length

        self.angle_a = 0  # angle between knee and ankle
        self.angle_b = 0  # angle between hip and ankle
        self.angle_c = 0  # angle between hip and knee
        self.current_angle = 0  # angle between hip 0 and the line from hip to ankle
        self.current_length = 0  # length of the distance between hip and ankle
        self.init_starting_triangle()

    def init_starting_triangle(self):
        self.angle_b = - math.pi * self.side + self.knee.initial_position + self.knee_offset
        b_side = math.sqrt(self.knee_ankle_length * self.knee_ankle_length + self.hip_knee_length * self.hip_knee_length
                           - 2 * self.knee_ankle_length * self.hip_knee_length * math.cos(self.angle_b))  # law of cosines
        a_angle = math.asin(self.knee_ankle_length * math.sin(self.angle_b) / b_side)  # law of sines
        c_angle = math.asin(self.hip_knee_length * math.sin(self.angle_b) / b_side)  # law of sines

        self.current_angle = self.hip.initial_position - a_angle  # + self.hip_offset

        self.angle_a = a_angle * self.side
        self.angle_c = c_angle * -self.side
        self.current_length = b_side
        print("Initialization:: angles:", a_angle, self.angle_b, c_angle, "lengths:", self.knee_ankle_length, b_side, self.hip_knee_length)
        print("current angle:", self.current_angle, "hip position:", self.hip.joint.get_position())

    def update(self):
        pass

    def move_relative(self, length_difference, angle_difference, time):
        self.current_angle += angle_difference
        self.extend_relative(length_difference, time)
        print(self.current_angle)

    def move(self, length, angle, time):
        self.current_angle = angle
        self.extend_to(length, time)

    def extend_relative(self, length_difference, time):
        self.extend_to(self.current_length + length_difference, time)

    def extend_to(self, length, time):
        assert 0 < length <= self.length_upper_limit  # leg length is within limits

        # abc corresponds to my notes; a is the lower leg, b is the desired length and c is the upper leg
        c = self.hip_knee_length  # this should probably be refactored later (insert directly)
        a = self.knee_ankle_length
        b = length
        angles = self.get_list_of_angles(a, b, c)  # angles; 0 top angle, 1 knee angle, 2 ankle angle
        # !!! angles has to be side adjusted !!!

        # calculate motor positions  // this is correct, don't change it
        hip_position = -self.current_angle - angles[0] * self.side
        knee_position = (math.pi-angles[1]) * self.side - self.knee_offset
        ankle_position = -self.current_angle + angles[2] * self.side

        # issue command
        self.hip.reach_position_in_time(hip_position, time)
        self.knee.reach_position_in_time(knee_position, time)
        self.ankle.reach_position_in_time(ankle_position, time)

        # update values
        self.current_length = length
        self.angle_a = angles[0]
        self.angle_c = -angles[2]

    def get_list_of_angles(self, a, b, c):
        return [self.angle_from_sides(b, c, a), self.angle_from_sides(c, a, b), self.angle_from_sides(a, b, c)]

    @staticmethod
    def angle_from_sides(a, b, c):
        """a, b, c being the sides of a triangle, returns the angle opposite of c"""
        return math.acos((a*a+b*b-c*c)/(2*a*b))  # cos^-1(law of cosines)

    def null(self):
        self.hip.joint.set_position(self.hip_offset)
        self.knee.joint.set_position(self.knee_offset)
        self.ankle.joint.set_position(self.ankle_offset)
