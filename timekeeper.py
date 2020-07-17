import time
from simulation import Simulation


# keeps the sim in realtime (more reasonable I-Controller solution)
# does not support switching physics sim frequency on the fly (don't know if anyone would do that, just don't do it).
class TimeFixer:
    def __init__(self, sim: Simulation):
        self.sim = sim

        self.pre_step_time = time.time_ns()  # when the last step began
        self.post_step_time = time.time_ns()  # when the last step ended

        self.desired_loop_time = 1000000000 * sim.time_step  # desired time for the entire loop
        self.actual_loop_time = 0  # actual time of the last loop

        self.error_over_time = 0  # the integral, i.e tracks the total error

    # this is the main method to use, it calls the sim step and handles timing.
    def physics_step(self):
        self.pre_step()
        self.sim.step()
        self.post_step()

    # reports how much time you are not using between physics steps
    def physics_step_report_time_left(self):
        self.time_left()
        self.physics_step()

    # Should be called before the sim step
    def pre_step(self):
        #  (how long the loop should be -how long the calculations took -integral) converted from nanoseconds to seconds
        sleep_time = (self.desired_loop_time - (time.time_ns() - self.post_step_time) - self.error_over_time)/1000000000
        if sleep_time > 0:
            time.sleep(sleep_time)  # some time has passed since time.time_ns() but  error_over_time will correct that
        self.pre_step_time = time.time_ns()  # record time

    # Should be called after the sim step
    def post_step(self):
        # updating times and calculating the last loop duration
        old_post_step_time = self.post_step_time
        self.post_step_time = time.time_ns()
        self.actual_loop_time = self.post_step_time - old_post_step_time

        error = self.actual_loop_time - self.desired_loop_time  # calculate the error of this loop
        self.error_over_time += error  # add the error to the integral

    # prints and returns how much percent of your time have passed since the last step (last call of postStep)
    def time_left(self):
        t = (self.post_step_time - time.time_ns()) / self.desired_loop_time * 100
        print(t, "% time used between physics steps")
        return t

    # optional use just before loop starts if time between __init__ and the first step is longer than desired_loop_time
    def set_last_time_step(self):
        self.post_step_time = time.time_ns()


# keeps the sim in realtime (mostly overkill and not used)
# does not support switching physics sim frequency on the fly
class Timekeeper:
    def __init__(self, sim: Simulation):
        self.sim = sim
        self.post_step_time = time.time_ns()  # when the last step ended
        self.pre_step_time = time.time_ns()  # when the last step began

        self.desired_loop_time = 1000000000 * sim.time_step   # desired time for the entire loop
        self.actual_loop_time = 0  # actual time the timekeeper achieved

        self.delay = 0  # actual-desired loop time
        self.average_error = 0

        self.average_step_duration = 0  # estimate how much time a step takes (most times very small to 0)

    def physics_step(self):
        self.pre_step()
        self.sim.step()
        self.post_step()

    # if timeUse = True reports how much time you are using between physics steps
    def physics_step_report_time_left(self):
        self.time_left()
        self.physics_step()

    # Should be called before the sim step
    def pre_step(self):
        time.sleep((self.desired_loop_time-(time.time_ns()-self.post_step_time)-self.delay) / 1000000000)  # nano->sec
        self.pre_step_time = time.time_ns()

    # Should be called after the sim step
    def post_step(self):
        old_post_step_time = self.post_step_time
        self.post_step_time = time.time_ns()
        self.actual_loop_time = self.post_step_time - old_post_step_time

        error = self.actual_loop_time - self.desired_loop_time
        self.average_error = (error + self.average_error) / 2
        cutoff = False
        noise_cutoff = 100000
        if self.average_error + noise_cutoff > error > self.average_error - noise_cutoff:  # ignore error_over_time edge-cases
            self.delay += error/2  # TODO write a nifty comment here
            cutoff = True

        self.average_step_duration = (self.average_step_duration + (self.post_step_time - self.pre_step_time)) / 2

        print("error_over_time:", int(error), "\tdesired:", int(self.desired_loop_time), "\tactual:", int(self.actual_loop_time), "\tdelay:", int(self.delay), "\ta-d:", int(self.actual_loop_time - self.delay), "\t", cutoff, "\t", int(self.average_error))

    # prints and returns how much percent of your time have passed since the last step (last call of postStep)
    def time_left(self):
        t = (self.post_step_time - time.time_ns()) / self.desired_loop_time * 100
        print(t, "% time used between physics steps")
        return t

    # use just before loop if the time between __init__ and the first step is longer than desired_loop_time
    def set_last_time_step(self):
        self.post_step_time = time.time_ns()
