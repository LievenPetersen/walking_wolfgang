import Simulation
import timekeeper
import random
import pybullet


if __name__ == "__main__":

    print("loading...")
    sim = Simulation.Simulation(True)
    print("simulation loaded\n")
    physicsClientId = sim.client_id
    keeper = timekeeper.Timefixer()

    jointList = []
    for joint in sim.joints:
        jointList.append(joint)

    while pybullet.isConnected(physicsClientId):
        key = jointList[random.randint(0, len(jointList)-1)]
        j = sim.joints.get(key)

        if isinstance(j, Simulation.Joint):
            j.set_position(random.randrange(int(j.lowerLimit), int(j.upperLimit)))
            # print(key)
        else:
            print(j.__str__())

        keeper.physicsStep(sim)

    print("\n--simulation terminated--")
