import pybullet
import simulation

import wolfgang_control
import timekeeper
import main

if __name__ == "__main__":
    s = main.Scheduler()
    s.run_sim()


class Scheduler:
    def __init__(self):
        print("loading...")
        self.sim = simulation.Simulation(True)
        print("simulation loaded\n")

        self.physicsClientId = self.sim.client_id

        self.control_system = wolfgang_control.SpasmBot(self.sim)
        self.keeper = timekeeper.Timefixer()

    def run_sim(self):
        while pybullet.isConnected(self.physicsClientId):
            self.control_system.step(self.sim)  # move the bot
            self.keeper.physicsStep(self.sim)  # step physics

        print("\n--simulation terminated--")
