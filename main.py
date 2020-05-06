import pybullet

import main
import simulation
import wolfgang_control
import timekeeper
import sim_interface

if __name__ == "__main__":
    s = main.Scheduler()
    s.run_sim()


class Scheduler:
    def __init__(self):
        print("loading...")
        self.sim = simulation.Simulation(True)
        self.physicsClientId = self.sim.client_id
        print("simulation loaded\n")

        self.interface = sim_interface.SimInterface(self.sim)
        self.control_system = wolfgang_control.SpasmBot(self.sim)
        self.keeper = timekeeper.Timefixer(self.sim)

        #  print(self.interface.get_foot_pressure()[0].left_back)

    def run_sim(self):
        while pybullet.isConnected(self.physicsClientId):
            self.control_system.step(self.sim)  # move the bot
            self.keeper.physicsStep(self.sim)  # step physics

        print("\n--simulation terminated--")
