import pybullet

import simulation
import bots
import timing
import sim_interface

# switch between bots here!


class Scheduler:
    def __init__(self):
        print("loading...")
        self.sim = simulation.Simulation(True)
        self.physicsClientId = self.sim.client_id
        print("simulation loaded\n")

        self.interface = sim_interface.SimInterface(self.sim)
        self.timer = timing.TimeFixer(self.sim)

        # chose the control-system for the bot. TODO: make switching more convenient i.e. switching on the fly possible
        self.control_system = bots.FastMoBot(self.interface)  # bots.<CurrentBot>(self.interface)

    def run_sim(self):
        while pybullet.isConnected(self.physicsClientId):
            if not self.sim.paused:
                self.control_system.step()  # updates the bots' control-system
            self.timer.physics_step()  # step physics

        print("\n--simulation terminated--")


if __name__ == "__main__":
    scheduler = Scheduler()
    scheduler.run_sim()
