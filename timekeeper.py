import time


# keeps the sim in realtime (mostly overkill)
class Timekeeper:
    def __init__(self):
        self.postStepTime = time.time_ns()  # when the last step ended
        self.preStepTime = time.time_ns()  # when the last step began

        self.desiredLoopTime = 1000000000 / 240  # desired time for the entire loop
        self.actualLoopTime = 0  # actual time the timekeeper achieved

        self.delay = 0  # actual-desired loop time
        self.averageError = 0

        self.averageStepDuration = 0  # estimate how much time a step takes (most times very small to 0)

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
        time.sleep((self.desiredLoopTime - (time.time_ns() - self.postStepTime) - self.delay)/1000000000)  # nano->sec
        self.preStepTime = time.time_ns()

    # Should be called after the sim step
    def postStep(self):
        oldPostStepTime = self.postStepTime
        self.postStepTime = time.time_ns()
        self.actualLoopTime = self.postStepTime - oldPostStepTime

        error = self.actualLoopTime - self.desiredLoopTime
        self.averageError = (error + self.averageError)/2

        if self.averageError*2 > error:  # sometimes the error errs...
            self.delay = (self.actualLoopTime - self.desiredLoopTime)/4 + self.delay  # TODO write a nifty comment here

        # calculate averageStepDuration
        # not really average but the average of the current duration and the averageStepDuration
        self.averageStepDuration = (self.averageStepDuration + (self.postStepTime - self.preStepTime)) / 2
        # print("expected step duration: ", self.averageStepDuration)
        # print("error: ", self.actualLoopTime - self.desiredLoopTime, " out of ", self.desiredLoopTime, "\t", self.actualLoopTime)

        print("error:", error, "\tdesired:", self.desiredLoopTime, "\tactual:", self.actualLoopTime, "\tdelay:", self.delay, "\ta-d:", self.actualLoopTime-self.delay)

    # prints and returns how much percent of your time have passed since the last step (last call of postStep)
    def timeLeft(self):
        t = (self.postStepTime - time.time_ns()) / self.desiredLoopTime * 100
        print(t, "% time used between physics steps")
        return t

    # use just before loop if the time between __init__ and the first step is longer than desiredLoopTime
    def setLastStep(self):
        self.postStepTime = time.time_ns()
