import simulation
import random
import math


class Bot:

    @classmethod
    def step(cls, sim):
        """is called once per physics tic"""
        return


class SpasmBot(Bot):
    def __init__(self, sim):
        self.joint_list = []
        for joint in sim.joints:
            self.joint_list.append(joint)

    def step(self, sim):
        key = self.joint_list[random.randint(0, len(self.joint_list) - 1)]
        j = sim.joints.get(key)

        if isinstance(j, simulation.Joint):
            j.set_position(random.randrange(math.ceil(j.lowerLimit), math.floor(j.upperLimit)))
            # print(key)
        else:
            print(j.__str__())
