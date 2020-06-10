import pybullet

import simulation
import bots
import timekeeper
import sim_interface


class Scheduler:
    def __init__(self):
        print("loading...")
        self.sim = simulation.Simulation(True)
        self.physicsClientId = self.sim.client_id
        print("simulation loaded\n")

        self.interface = sim_interface.SimInterface(self.sim)
        self.timer = timekeeper.Timefixer(self.sim)

        # chose the control-system for the bot. TODO: make switching more convenient i.e. switching on the fly possible
        self.control_system = bots.SlowMoBot(self.interface)

    def run_sim(self):
        while pybullet.isConnected(self.physicsClientId):
            if not self.sim.paused:
                self.control_system.step()  # updates the bots' control-system
            self.timer.physicsStep(self.sim)  # step physics

        print("\n--simulation terminated--")


if __name__ == "__main__":
    scheduler = Scheduler()
    scheduler.run_sim()
