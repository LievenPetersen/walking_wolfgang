import random
import math

import simulation
from sim_interface import SimInterface


class MotorController:
    """provides abstract control for a given joint"""
    def __init__(self, name, interface: SimInterface):
        assert interface.get_joint(name) is not None  # joint exists (i.e. did you spell it right?)

        self.name = name  # name of the joint
        self.joint = interface.get_joint(name)
        self.initial_position = interface.get_initial_joint_position(name)

    def step(self):
        print("wheeeeeeeeeeeee")
        pass

    def goto_initial_position(self):
        self.joint.set_position(math.radians(self.initial_position))

    def reach_position_in_time(self, position, time_to_reach):
        delta_position = self.joint.get_position() - position
        if time_to_reach > 0:
            speed = delta_position/time_to_reach
        else:
            speed = self.joint.max_velocity
        self.joint.set_position(velocity=math.radians(speed), position=math.radians(position))

    def get_name(self):
        return self.name


class Bot:
    motor_controllers = []
    interface: SimInterface

    @classmethod
    def __init__(cls, interface: SimInterface):
        cls.interface = interface

    @classmethod
    def update_controllers(cls):
        """needs to be called at the end of each physics step"""
        # update each controller
        for controller in cls.motor_controllers:
            controller.step()

    @classmethod
    def create_controller(cls, name):
        controller = MotorController(name, cls.interface)
        cls.motor_controllers.append(controller)
        return controller


class SpasmBot(Bot):
    def __init__(self, interface: SimInterface):
        super().__init__(interface)

        self.joint_list = []
        for joint in self.interface.simulation.joints:
            self.joint_list.append(joint)

    def step(self):
        key = self.joint_list[random.randint(0, len(self.joint_list) - 1)]
        j = self.interface.simulation.joints.get(key)

        if isinstance(j, simulation.Joint):
            j.set_position(random.randrange(math.ceil(j.lowerLimit), math.floor(j.upperLimit)))
        else:
            print(j.__str__())


class SquadBot(Bot):
    def __init__(self, interface: SimInterface):
        super().__init__(interface)
        self.i = 0
        self.r_hip_pitch = self.create_controller("RHipPitch")

        print(interface.simulation.joints)

    def step(self):
        if self.i % 200 == 0:
            self.r_hip_pitch.goto_initial_position()
        if self.i % 200 == 100:
            self.r_hip_pitch.goto_initial_position()
        self.i += 1

        self.update_controllers()  # give all controllers the chance to update (is a @classmethod of Bot)
