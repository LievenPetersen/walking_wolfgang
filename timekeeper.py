import time
import _collections


# keeps the sim in realtime (mostly overkill)
class Timekeeper:
    def __init__(self):
        self.post_step_time = time.time_ns()  # when the last step ended
        self.preStepTime = time.time_ns()  # when the last step began

        self.desired_loop_time = 1000000000 / 240  # desired time for the entire loop
        self.actual_loop_time = 0  # actual time the timekeeper achieved

        self.delay = 0  # actual-desired loop time
        self.average_error = 0

        self.average_step_duration = 0  # estimate how much time a step takes (most times very small to 0)

    def physicsStep(self, sim):
        self.preStep()
        sim.step()
        self.postStep()

    # if timeUse = True reports how much time you are using between physics steps
    def physicsStepT(self, sim):
        self.timeLeft()
        self.physicsStep(sim)

    # Should be called before the sim step
    def preStep(self):
        time.sleep((self.desired_loop_time - (time.time_ns() - self.post_step_time) - self.delay) / 1000000000)  # nano->sec
        self.preStepTime = time.time_ns()

    # Should be called after the sim step
    def postStep(self):
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

        self.average_step_duration = (self.average_step_duration + (self.post_step_time - self.preStepTime)) / 2

        print("error_over_time:", int(error), "\tdesired:", int(self.desired_loop_time), "\tactual:", int(self.actual_loop_time), "\tdelay:", int(self.delay), "\ta-d:", int(self.actual_loop_time - self.delay), "\t", cutoff, "\t", int(self.average_error))

    # prints and returns how much percent of your time have passed since the last step (last call of postStep)
    def timeLeft(self):
        t = (self.post_step_time - time.time_ns()) / self.desired_loop_time * 100
        print(t, "% time used between physics steps")
        return t

    # use just before loop if the time between __init__ and the first step is longer than desired_loop_time
    def setLastStep(self):
        self.post_step_time = time.time_ns()


# keeps the sim in realtime (mostly overkill)
class Timefixer:
    def __init__(self):
        self.post_step_time = time.time_ns()  # when the last step ended
        self.preStepTime = time.time_ns()  # when the last step began

        self.desired_loop_time = 1000000000 / 240  # desired time for the entire loop
        self.actual_loop_time = 0  # actual time the timekeeper achieved

        self.average_error = 0
        self.error_over_time = 0

        self.average_step_duration = 0  # estimate how much time a step takes (most times very small to 0)

        self.looptimes = _collections.deque()

    def physicsStep(self, sim):
        self.preStep()
        sim.step()
        self.postStep()

    # if timeUse = True reports how much time you are using between physics steps
    def physicsStepT(self, sim):
        self.timeLeft()
        self.physicsStep(sim)

    # Should be called before the sim step
    def preStep(self):
        time.sleep((self.desired_loop_time - (time.time_ns() - self.post_step_time) - self.error_over_time) / 1000000000)  # nano->sec
        self.preStepTime = time.time_ns()

    # Should be called after the sim step
    def postStep(self):
        old_post_step_time = self.post_step_time
        self.post_step_time = time.time_ns()
        self.actual_loop_time = self.post_step_time - old_post_step_time

        error = self.actual_loop_time - self.desired_loop_time
        self.error_over_time += error

        """
        # debug
        if len(self.looptimes) > 200:
            self.looptimes.popleft()
        self.looptimes.append(self.actual_loop_time)

        average_looptime = 0
        for x in self.looptimes:
            average_looptime += x
        average_looptime /= len(self.looptimes)
        
        """
        # print(int(self.error_over_time), int(error), int(self.error_over_time - error), int(average_looptime))
        # use for debug

    # prints and returns how much percent of your time have passed since the last step (last call of postStep)
    def timeLeft(self):
        t = (self.post_step_time - time.time_ns()) / self.desired_loop_time * 100
        print(t, "% time used between physics steps")
        return t

    # optional use just before loop if the time between __init__ and the first step is longer than desired_loop_time
    def setLastStep(self):
        self.post_step_time = time.time_ns()
