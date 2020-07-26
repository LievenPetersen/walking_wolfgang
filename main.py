import pybullet

import simulation
import bots
import timing
import sim_interface


class Scheduler:
    """Gets everything running, switch between bots here"""
    def __init__(self):
        """initializes simulation, interface, timer and control-system"""
        print("loading...")
        self.sim = simulation.Simulation(True)
        self.physicsClientId = self.sim.client_id
        print("simulation loaded\n")

        self.interface = sim_interface.SimInterface(self.sim)
        self.timer = timing.Timer(self.sim)

        # chose the control-system for the bot.
        self.control_system = bots.SlowMoBot(self.interface)  # bots.<CurrentBot>(self.interface)

    def run_sim(self):
        """starts the main loop of the simulation"""
        while pybullet.isConnected(self.physicsClientId):
            if not self.sim.paused:
                self.control_system.step()  # updates the bots' control-system
            self.timer.step_simulation()  # step physics

        print("\n--simulation terminated--")


if __name__ == "__main__":
    scheduler = Scheduler()
    scheduler.run_sim()
