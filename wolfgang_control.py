import random
import math

import abc

import simulation
from sim_interface import SimInterface


class MotorController:
    """provides abstract control for a given joint"""

    def __init__(self, name, interface: SimInterface):
        assert interface.get_joint(name) is not None  # joint exists (i.e. did you spell it right?)

        self.interface = interface

        self.name = name  # name of the joint
        self.joint = interface.get_joint(name)
        self.initial_position = math.radians(interface.get_initial_joint_position(name))

        self.radiant_error_tolerance = 0.001  # if position-error is greater than error_tolerance vel. is recalculated

        self.current_start_position = 0  # were the current move started from
        self.current_target_position = 0  # were the current move should end
        self.current_speed = 0  # how fast the current move should be
        self.current_start_time = 0  # simulation time of the start of the movement
        self.current_target_time = 0  # at which simulation time the target should be reached

    def update(self):
        if self.current_target_time is not 0 and self.current_target_time <= self.interface.simulation.time:
            self.joint.set_position(self.current_target_position)
            self.current_speed = 0
            self.current_target_time = 0
            print("reached", self.get_name(), "position with error",
                  self.joint.get_position() - self.current_target_position)

        # if the positional error is greater than the error tolerance:
        elif self.current_target_time is not 0 \
                and abs(self.joint.get_position() - self.get_predicted_position()) > self.radiant_error_tolerance:

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

    def get_name(self):
        return self.name

    def get_predicted_position(self):
        return self.current_start_position + self.get_duration_of_current_movement() * self.current_speed

    def get_duration_of_current_movement(self):
        return self.interface.simulation.time - self.current_start_time


class Leg:
    def __init__(self, side):
        self.current_length = 0
        pass

    def extend_relative(self, length_difference, time):
        self.extend_to(self.current_length + length_difference, time)

    def extend_to(self, length, time):
        pass


class Bot(abc.ABC):
    motor_controllers = []
    interface: SimInterface

    def __init__(self, interface: SimInterface):
        self.interface = interface

    def step(self):
        """This method should be called every time_step to update the Bot"""
        self.update()  # custom update specified by the bot
        self.update_controllers()  # streamlined update for all controllers

    @abc.abstractmethod
    def update(self):
        pass

    def update_controllers(self):
        """needs to be called at the end of each physics step"""
        # update each controller
        for controller in self.motor_controllers:
            controller.update()

    def create_controller(self, name):
        controller = MotorController(name, self.interface)
        self.motor_controllers.append(controller)
        return controller


class SpasmBot(Bot):
    def __init__(self, interface: SimInterface):
        super().__init__(interface)

        self.joint_list = []
        for joint in self.interface.simulation.joints:
            self.joint_list.append(joint)

    def update(self):
        key = self.joint_list[random.randint(0, len(self.joint_list) - 1)]
        j = self.interface.simulation.joints.get(key)

        if isinstance(j, simulation.Joint):
            j.set_position(random.randrange(math.ceil(j.lowerLimit), math.floor(j.upperLimit)))
        else:
            print(j.__str__())


class SquatBot(Bot):
    def __init__(self, interface: SimInterface):
        super().__init__(interface)
        self.i = 0

        # right leg
        self.r_hip_pitch = self.create_controller("RHipPitch")
        self.r_knee = self.create_controller("RKnee")
        self.r_ankle_pitch = self.create_controller("RAnklePitch")
        # left leg
        self.l_hip_pitch = self.create_controller("LHipPitch")
        self.l_knee = self.create_controller("LKnee")
        self.l_ankle_pitch = self.create_controller("LAnklePitch")

        # print(interface.simulation.joints)

    def update(self):
        if self.i % (240 * 10) == 240 * 1:
            self.r_hip_pitch.reach_position_in_time(self.r_hip_pitch.initial_position - 0.2 * math.pi, 3)
            self.r_knee.reach_position_in_time(self.r_knee.initial_position + 0.4 * math.pi, 3)
            self.r_ankle_pitch.reach_position_in_time(self.r_ankle_pitch.initial_position + 0.2 * math.pi, 3)

            self.l_hip_pitch.reach_position_in_time(self.l_hip_pitch.initial_position + 0.2 * math.pi, 3)
            self.l_knee.reach_position_in_time(self.l_knee.initial_position - 0.4 * math.pi, 3)
            self.l_ankle_pitch.reach_position_in_time(self.l_ankle_pitch.initial_position - 0.2 * math.pi, 3)

        elif self.i % (240 * 10) == 240 * 6:
            self.r_hip_pitch.reach_position_in_time(self.r_hip_pitch.initial_position, 3)
            self.r_knee.reach_position_in_time(self.r_knee.initial_position, 3)
            self.r_ankle_pitch.reach_position_in_time(self.r_ankle_pitch.initial_position, 3)

            self.l_hip_pitch.reach_position_in_time(self.l_hip_pitch.initial_position, 3)
            self.l_knee.reach_position_in_time(self.l_knee.initial_position, 3)
            self.l_ankle_pitch.reach_position_in_time(self.l_ankle_pitch.initial_position, 3)

        self.i += 1


class TorgeBot(Bot):
    def __init__(self, interface: SimInterface):
        super().__init__(interface)
        print(interface.simulation.joints)
        self.head_pitch = MotorController("HeadTilt", interface)

        self.i = 0

    def update(self):
        if self.head_pitch.joint.get_position() + self.head_pitch.joint.get_velocity()\
                * self.interface.simulation.time_step >= self.head_pitch.joint.upperLimit \
                or self.head_pitch.joint.get_position() + self.head_pitch.joint.get_velocity()\
                * self.interface.simulation.time_step <= self.head_pitch.joint.lowerLimit:
            self.head_pitch.joint.set_position(self.head_pitch.joint.get_position())
            print("hold")

        if self.i % (240 * 4) == 240 * 0:
            self.head_pitch.joint.set_torque(-0.01)
            print("-")
        elif self.i % (240 * 4) == 240 * 2:
            self.head_pitch.joint.set_torque(0.01)
            print("+")

        self.i += 1
