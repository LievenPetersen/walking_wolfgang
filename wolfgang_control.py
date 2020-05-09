import random
import math

import simulation
from sim_interface import SimInterface


class Bot:
    def __init__(self, interface: SimInterface):
        self.interface = interface

    @classmethod
    def step(cls, sim):
        """is called once per physics tic"""
        print("Warning: chosen control-system does not provide a control loop")
        return


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
            # print(key)
        else:
            print(j.__str__())


class SquadBot(Bot):
    def __init__(self, interface: SimInterface):
        super().__init__(interface)
        self.i = 0

    def step(self):
        self.i += 1
