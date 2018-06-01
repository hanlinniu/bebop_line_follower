#!/usr/bin/python
'''

__Author_ = Lowyi
__Email__ = MR.LowBattery@gmail.com
__Team__  = MRL_UAV

'''

import time
import roslib

roslib.load_manifest('bebop_line_follower')
# import sys
import rospy
from std_msgs.msg import String, Int32
from pid import PID
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from math import *
from bebop_msgs.msg import Ardrone3PilotingStateAttitudeChanged, Ardrone3PilotingStateSpeedChanged


class controller:
    ''' Handles the PID control in every axis of the drone deppending on the flight mode '''

    def __init__(self):
        self.roll_control = PID(0.1, 0, 1)
        self.altitude_control = PID(1, 0, 0)
        self.pitch_control = PID(0.1, 0, 0)
        self.yaw_control = PID(1, 0, 0.1)
        self.command = Twist()
        self.first_time = 0
        self.state_altitude = 0
        self.takeoff_time = 0
        self.vy = 0
        self.last_vy = 0
        self.last_time = 0
        self.keyboard_mode = 0
        self.rotX = 0

        self.image_pos = rospy.Subscriber("data", Quaternion, self.callback)

        self.droneRPY = rospy.Subscriber('/bebop/states/ardrone3/PilotingState/AttitudeChanged',
                                         Ardrone3PilotingStateAttitudeChanged, self.ReceiveRPY)

        self.droneSpeed = rospy.Subscriber('/bebop/states/ardrone3/PilotingState/SpeedChanged',
                                           Ardrone3PilotingStateSpeedChanged, self.ReceiveSpeed)

        self.pubCommand = rospy.Publisher('bebop/cmd_vel', Twist, queue_size=10)

        self.pub_test = rospy.Publisher('/pid_teste', String, queue_size=10)

    def callback(self, Quaternion):

        if Quaternion.w ==1:

            self.line_follower(Quaternion)

    def ReceiveRPY(self, Ardrone3PilotingStateAttitudeChanged):

        self.rotX = Ardrone3PilotingStateAttitudeChanged.roll * (pi / 180)

    def ReceiveSpeed(self, Ardrone3PilotingStateSpeedChanged):

        self.vy = Ardrone3PilotingStateSpeedChanged.speedX / 1000

    def SetCommand(self, roll=0, pitch=0, yaw_velocity=0, z_velocity=0):

        # set the current command
        self.command.linear.x = pitch
        self.command.linear.y = roll
        self.command.linear.z = z_velocity
        self.command.angular.z = yaw_velocity

    def SendCommand(self):
        self.pubCommand.publish(self.command)

    def line_follower(self, Quaternion):
        # controller for the line follower mode
        if self.first_time == 0:
            self.roll_control.setConstants(0.1, 0, 1)
            self.yaw_control.setConstants(1, 0, 0.1)
            self.first_time = 1

        x = -Quaternion.x
        detecting = int(Quaternion.y)
        angle = -Quaternion.z

        offset = int(856 * tan(self.rotX) / (tan(0.52 + self.rotX) + tan(0.52 - self.rotX)))

        if detecting != 0:
            x = (x + offset) / 480.0
            pitch_vel = 0.1
        else:
            pitch_vel = 0
            x /= 480.0

        if x == 0:
            x = self.roll_control.last_error
            self.yaw_control.last_error = 0

        while (time.time() - self.takeoff_time < 8):
            self.roll_control.reset()

        roll_output = self.roll_control.update(x)
        yaw_output = self.yaw_control.update(angle)

        self.SetCommand(roll_output, pitch_vel, yaw_output, 0)

        if (time.time() - self.takeoff_time > 8):
            self.SendCommand()

        self.pub_test.publish(str(roll_output) + " " + str(pitch_vel) + " " + str(yaw_output))


def main(args):
    ic = controller()
    rospy.init_node('controller', anonymous=True)

    rospy.spin()


if __name__ == '__main__':
    #
    import sys

    main(sys.argv)
