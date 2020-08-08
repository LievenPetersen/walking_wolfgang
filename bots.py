import random
import math
import abc

import simulation
from sim_interface import SimInterface
import controllers


class Bot(abc.ABC):

    def __init__(self, interface: SimInterface):
        self.__leg_controllers = []
        self.__motor_controllers = []
        self.interface = interface

    def step(self):
        """This method should be called every time_step to update the Bot"""
        self.update()  # abstract update method, implemented by the bot.
        self.update_controllers()  # streamlined update for all controllers

    @abc.abstractmethod
    def update(self):
        """This method has to be implemented by every bot and contains the core control loop"""
        pass

    def update_controllers(self):
        """needs to be called at the end of each physics step"""
        for leg in self.__leg_controllers:
            leg.update()
        # update each controller
        for controller in self.__motor_controllers:
            controller.update()

    def create_leg_controller(self, side: int, smooth: bool = False):
        """creates a leg controller for one side of the robot.
        side == 1 for the right side
        side == -1 for the left side
        Also creates the controllers for the joints and assigns them.
        (they can be accessed through the leg object later)
        if smooth is true a SmoothMotorController is used.
        """
        assert side is 1 or side is -1  # +1 means right, -1 means left side.

        if smooth:
            if side == -1:
                hip_pitch = self.create_smooth_motor_controller("LHipPitch")
                knee = self.create_smooth_motor_controller("LKnee")
                ankle_pitch = self.create_smooth_motor_controller("LAnklePitch")

            else:
                hip_pitch = self.create_smooth_motor_controller("RHipPitch")
                knee = self.create_smooth_motor_controller("RKnee")
                ankle_pitch = self.create_smooth_motor_controller("RAnklePitch")

        else:
            if side == -1:
                hip_pitch = self.create_motor_controller("LHipPitch")
                knee = self.create_motor_controller("LKnee")
                ankle_pitch = self.create_motor_controller("LAnklePitch")

            else:
                hip_pitch = self.create_motor_controller("RHipPitch")
                knee = self.create_motor_controller("RKnee")
                ankle_pitch = self.create_motor_controller("RAnklePitch")

        leg = controllers.Leg(hip_pitch, knee, ankle_pitch, side, self.interface)
        self.__leg_controllers.append(leg)
        return leg

    def create_smooth_motor_controller(self, name):
        """creates and assigns a SmoothMotorController.
        Can be used right away, no further actions needed
        """
        controller = controllers.SmoothMotorController(name, self.interface)
        self.__motor_controllers.append(controller)
        return controller

    def create_motor_controller(self, name):
        """creates and assigns a MotorController.
        Can be used right away, no further actions needed
        """
        controller = controllers.MotorController(name, self.interface)
        self.__motor_controllers.append(controller)
        return controller


class SpasmBot(Bot):
    """spastic bot that executes a random movement every tick.
    Used for early testing of the simulation and environment
    """
    def __init__(self, interface: SimInterface):
        super().__init__(interface)
        self.joint_list = []
        for joint in self.interface.simulation.joints:
            self.joint_list.append(joint)

    def update(self):
        key = self.joint_list[random.randint(0, len(self.joint_list) - 1)]
        j = self.interface.simulation.joints.get(key)

        if isinstance(j, simulation.Joint):
            j.set_position(random.randrange(math.ceil(j.lowerLimit), math.floor(j.upperLimit)))
        else:
            print(j)


class SquatBot(Bot):
    """A bot that does squads
    Used to test the MotorController and the updating methods for the bot
    """
    def __init__(self, interface: SimInterface):
        super().__init__(interface)
        self.i = 0

        # right leg
        self.r_hip_pitch = self.create_motor_controller("RHipPitch")
        self.r_knee = self.create_motor_controller("RKnee")
        self.r_ankle_pitch = self.create_motor_controller("RAnklePitch")
        # left leg
        self.l_hip_pitch = self.create_motor_controller("LHipPitch")
        self.l_knee = self.create_motor_controller("LKnee")
        self.l_ankle_pitch = self.create_motor_controller("LAnklePitch")

        # print(interface.simulation.joints)

    def update(self):
        time = 0.5
        if self.i % (240 * time * 4) == 240 * time * 1:
            self.r_hip_pitch.reach_position_in_time(self.r_hip_pitch.initial_position - 0.2 * math.pi, time)
            self.r_knee.reach_position_in_time(self.r_knee.initial_position + 0.4 * math.pi, time)
            self.r_ankle_pitch.reach_position_in_time(self.r_ankle_pitch.initial_position + 0.2 * math.pi, time)

            self.l_hip_pitch.reach_position_in_time(self.l_hip_pitch.initial_position + 0.2 * math.pi, time)
            self.l_knee.reach_position_in_time(self.l_knee.initial_position - 0.4 * math.pi, time)
            self.l_ankle_pitch.reach_position_in_time(self.l_ankle_pitch.initial_position - 0.2 * math.pi, time)

        elif self.i % (240 * time * 4) == 240 * time * 3:
            self.r_hip_pitch.reach_position_in_time(self.r_hip_pitch.initial_position, time)
            self.r_knee.reach_position_in_time(self.r_knee.initial_position, time)
            self.r_ankle_pitch.reach_position_in_time(self.r_ankle_pitch.initial_position, time)

            self.l_hip_pitch.reach_position_in_time(self.l_hip_pitch.initial_position, time)
            self.l_knee.reach_position_in_time(self.l_knee.initial_position, time)
            self.l_ankle_pitch.reach_position_in_time(self.l_ankle_pitch.initial_position, time)

        self.i += 1


class SquatBotLegController(Bot):
    """Later iteration of SquadBot, that uses the Leg controller.
    Used to test... the leg controller!
    """
    def __init__(self, interface: SimInterface):
        super().__init__(interface)
        self.i = 0

        use_smooth_controller = True
        self.r_leg = self.create_leg_controller(1, use_smooth_controller)
        self.l_leg = self.create_leg_controller(-1, use_smooth_controller)

    def update(self):
        move_time = 2
        wait_time = 0.5
        if self.i % (240 * (move_time + wait_time) * 2) == 240 * wait_time:
            self.r_leg.extend_relative(-0.18, move_time)
            self.l_leg.extend_relative(-0.18, move_time)

        elif self.i % (240 * (move_time + wait_time) * 2) == 240 * (move_time + 2 * wait_time):
            self.r_leg.extend_relative(0.18, move_time)
            self.l_leg.extend_relative(0.18, move_time)
        self.i += 1


class SlowMoBot(Bot):
    """First Bot to perform static walking
    Designed to be slow, walking parameters can be tweaked in the update method.
    """
    def __init__(self, interface: SimInterface):
        super().__init__(interface)
        self.i = 0

        use_smooth_controller = True
        self.r_leg = self.create_leg_controller(1, use_smooth_controller)
        self.l_leg = self.create_leg_controller(-1, use_smooth_controller)

        self.r_hip_roll = self.create_smooth_motor_controller("RHipRoll")
        self.l_hip_roll = self.create_smooth_motor_controller("LHipRoll")
        self.r_ankle_roll = self.create_smooth_motor_controller("RAnkleRoll")
        self.l_ankle_roll = self.create_smooth_motor_controller("LAnkleRoll")

        self.walking = False

    def update(self):
        cycle_time = 6  # touch this
        quarter_time = cycle_time/6
        sway = 0.2  # and this. But the rest can be fragile
        move_time = quarter_time - 0.01  # *0.8  # there needs to be a small time gap between moves
        leg_angle = 0.2
        extend_length = 0.1

        if not self.walking:
            if self.i % (240 * cycle_time) == 0:
                # sway right
                self.sway(sway, move_time)
                # move torso upright
                self.r_leg.move_relative(0, 0, move_time)
                self.l_leg.move_relative(0, 0, move_time)

            elif self.i % (240 * cycle_time) == 240 * quarter_time * 1:
                # pick left leg up
                self.l_leg.move_relative(-extend_length, -leg_angle, move_time)
                self.walking = True

        else:
            if self.i % (240 * cycle_time) == 0:
                # sway right
                self.sway(sway, move_time)

            elif self.i % (240 * cycle_time) == 240 * quarter_time * 1:
                # pick left leg up
                self.r_leg.move(self.r_leg.current_length, 0, move_time)
                self.l_leg.move_relative(-extend_length, -leg_angle, move_time)

            elif self.i % (240 * cycle_time) == 240 * quarter_time * 2:
                self.sway(0, move_time)
                self.r_leg.move(self.r_leg.current_length, -leg_angle, move_time*0.95)
                self.l_leg.move_relative(extend_length, -leg_angle, move_time)

            elif self.i % (240 * cycle_time) == 240 * quarter_time * 3:
                # sway left
                self.sway(-sway, move_time)

            elif self.i % (240 * cycle_time) == 240 * quarter_time * 4:
                # pick right leg up
                self.r_leg.move_relative(-extend_length, leg_angle, move_time)
                self.l_leg.move(self.l_leg.current_length, 0, move_time)

            elif self.i % (240 * cycle_time) == 240 * quarter_time * 5:
                self.sway(0, move_time)
                self.r_leg.move_relative(extend_length, leg_angle, move_time)
                self.l_leg.move(self.r_leg.current_length, leg_angle, move_time*0.95)

        # keep this, trust me
        self.i += 1

    def sway(self, sway, time):
        self.r_hip_roll.reach_position_in_time(sway, time)
        self.l_hip_roll.reach_position_in_time(sway, time)
        self.r_ankle_roll.reach_position_in_time(sway, time)
        self.l_ankle_roll.reach_position_in_time(sway, time)


class FastMoBot(Bot):
    """Top secret experimental bot.
     Trying to go faster by eliminating 2 phases of SlowMoBot in which the bot stopped to sway.
     But don't tell anyone ;)"""
    def __init__(self, interface: SimInterface):
        super().__init__(interface)
        self.i = 0

        use_smooth_controller = True
        self.r_leg = self.create_leg_controller(1, use_smooth_controller)
        self.l_leg = self.create_leg_controller(-1, use_smooth_controller)

        self.r_hip_roll = self.create_smooth_motor_controller("RHipRoll")
        self.l_hip_roll = self.create_smooth_motor_controller("LHipRoll")
        self.r_ankle_roll = self.create_smooth_motor_controller("RAnkleRoll")
        self.l_ankle_roll = self.create_smooth_motor_controller("LAnkleRoll")

        self.walking = False

    def update(self):
        """It's not really working yet, but carefully resetting the sim (r) might get you some walking
        """
        cycle_time = 1.5
        quarter_time = cycle_time/4
        sway = 0.09
        move_time = quarter_time - 0.01  # *0.8
        leg_angle = 0.2
        extend_length = 0.1

        if not self.walking:
            if self.i % (240 * cycle_time) == 0:
                # sway right
                self.sway(sway, move_time)
                # move torso upright
                self.r_leg.move_relative(0, 0, move_time)
                self.l_leg.move_relative(0, 0, move_time)

            elif self.i % (240 * cycle_time) == 240 * quarter_time * 1:
                # pick left leg up
                self.r_leg.move(self.r_leg.current_length, -leg_angle, move_time*0.95)
                self.l_leg.move_relative(-extend_length, -leg_angle, move_time)
                self.walking = True
                self.i = 240 * 3

        else:
            if self.i % (240 * cycle_time) == 240 * quarter_time * 0:
                # pick left leg up
                self.r_leg.move(self.r_leg.current_length, 0, move_time)
                self.l_leg.move_relative(-extend_length, -leg_angle, move_time)

            elif self.i % (240 * cycle_time) == 240 * quarter_time * 1:
                # sway left
                self.sway(-sway, move_time)
                self.r_leg.move(self.r_leg.current_length, -leg_angle, move_time*0.95)
                self.l_leg.move_relative(extend_length, -leg_angle, move_time)

            elif self.i % (240 * cycle_time) == 240 * quarter_time * 2:
                # pick right leg up
                self.r_leg.move_relative(-extend_length, leg_angle, move_time)
                self.l_leg.move(self.l_leg.current_length, 0, move_time)

            elif self.i % (240 * cycle_time) == 240 * quarter_time * 3:
                self.sway(sway, move_time)
                self.r_leg.move_relative(extend_length, leg_angle, move_time)
                self.l_leg.move(self.r_leg.current_length, leg_angle, move_time*0.95)

        # keep this, trust me
        self.i += 1

    def sway(self, sway, time):
        self.r_hip_roll.reach_position_in_time(sway, time)
        self.l_hip_roll.reach_position_in_time(sway, time)
        self.r_ankle_roll.reach_position_in_time(sway, time)
        self.l_ankle_roll.reach_position_in_time(sway, time)


class TorgeBot(Bot):
    """You could call this a disbanded experiment.
    Trying torque control.
    turns out to be unpractical in the simulation due to joints getting stuck when over limit
    """
    def __init__(self, interface: SimInterface):
        super().__init__(interface)
        print(interface.simulation.joints)
        self.head_pitch = controllers.MotorController("HeadTilt", interface)

        self.i = 0

    def update(self):
        if self.head_pitch.joint.get_position() + math.radians(5) > self.head_pitch.joint.upperLimit\
                and self.head_pitch.joint.get_position() - math.radians(5) < self.head_pitch.joint.lowerLimit:
            self.head_pitch.joint.set_position(self.head_pitch.joint.get_position())
            print("hold")

        if self.i % (240 * 4) == 240 * 0:
            self.head_pitch.joint.set_torque(-1)
            print("-")
        elif self.i % (240 * 4) == 240 * 2:
            self.head_pitch.joint.set_torque(1)
            print("+")

        self.i += 1
