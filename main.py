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
        self.timer = timekeeper.Timefixer(self.sim)

        # chose the control-system for the bot. TODO: make switching more convenient i.e. switching on the fly possible
        self.control_system = wolfgang_control.SquadBot(self.interface)

    def run_sim(self):
        while pybullet.isConnected(self.physicsClientId):
            self.control_system.step()  # move the bot
            self.timer.physicsStep(self.sim)  # step physics

        print("\n--simulation terminated--")
