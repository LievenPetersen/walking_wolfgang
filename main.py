import Simulation
import time
import timekeeper
import random


if __name__ == "__main__":

    print("loading...")
    sim = Simulation.Simulation(True)
    keeper = timekeeper.Timekeeper()
    print("ready")

    jointList = []
    for joint in sim.joints:
        jointList.append(joint)

    print("running")

    while True:
        key = jointList[random.randint(0, len(jointList)-1)]
        j = sim.joints.get(key)

        if isinstance(j, Simulation.Joint):
            j.set_position(random.randrange(int(j.lowerLimit), int(j.upperLimit)))
            # print(key)
        else:
            print(j.__str__())

        keeper.physicsStep(sim)

    print("simulation terminated")
