#!/usr/bin/env python
import rospy
import math
import random
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from tf import transformations
from zigzagbot import ZigZagBot
from mw_utilities import first_non_nan
from mw_utilities import is_perpendicular


class MazeWanderer:
    """Subscribes to topics relating to bot state and issues movement commands until victory condition reached.
        See wander_maze_with_bot and move_bot for bulk of class's logic.

    """
    def __init__(self, bot, victory_grid):
        rospy.init_node('maze_wanderer')

        # subscribe to all the topics we need
        rospy.Subscriber('scan', LaserScan, MazeWanderer.scan_callback, bot)
        rospy.Subscriber('mobile_base/events/bumper', BumperEvent, MazeWanderer.bump_callback, bot)
        rospy.Subscriber('odom', Odometry, MazeWanderer.odometry_callback, bot)
        rospy.Subscriber('gazebo/model_states', ModelStates, MazeWanderer.gazebo_callback, bot)

        # power on moving and victory evaluation loop
        MazeWanderer.wander_maze_with_bot(bot, victory_grid)

    @staticmethod
    def scan_callback(scan_msg, bot):
        """Extract extreme left and right scan readings.

        :param scan_msg: msg of type sensor_msgs/LaserScan from topic /scan
        :param bot: instance of ZigZagBot used by a MazeWanderer instance
        :return:
        """
        ranges = scan_msg.ranges
        left = first_non_nan(ranges, True)
        right = first_non_nan(ranges, False)
        bot.left_scan = left
        bot.right_scan = right

    @staticmethod
    def bump_callback(bump_msg, bot):
        """Update bot state with notification that it has bumped into something.

        :param bump_msg: msg of type kobuki_msgs/BumperEvent from topic /mobile_base/events/bumper
        :param bot: instance of ZigZagBot used by a MazeWanderer instance
        :return:
        """
        if bump_msg:  # often returns NoneType for some reason
            if bool(bump_msg.state):  # only set if bumping is active -- bumping is toggled off in turning logic
                bot.is_bumping = True

    @staticmethod
    def odometry_callback(odom_msg, bot):
        """Update bot state with orientation in degrees (ranging from -180 to 180).

        :param odom_msg: msg of type nav_msgs/Odometry from topic /odom
        :param bot: instance of ZigZagBot used by a MazeWanderer instance
        :return:
        """
        quaternion = [odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y,
                      odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w]
        euler = transformations.euler_from_quaternion(quaternion)
        bot.orientation = math.degrees(euler[2])

    @staticmethod
    def gazebo_callback(model_state_msg, bot):
        bot.x = model_state_msg.pose[1].position.x
        bot.y = model_state_msg.pose[1].position.y
        # rospy.loginfo("bot position: {}, {}".format(bot.x, bot.y))

    @staticmethod
    def move_bot(bot, cmd_pub):
        """Chooses appropriate movement for bot state and builds msg to execute on passed command publisher.

            This is the primary method of the class as measured by the frequency of it being called and its
            importance to the purpose of the class (bumping through the maze until hitting the cube).

            Put simply, the control flow is:
                1.) A bump is register by bump_callback and the bot state is set to is_bumping = True
                    NOTE: Until both turning to face the wall head-on and then turning 90ish degrees are complete,
                    is_bumping is left True. Essentially, is_bumping is used to indicate that the sequence of
                    commands that need to happen after a wall collision are either still in process or are complete.
                2.) The bot turns until it faces the wall head-on, checking to see if it is already perpendicular
                    or if it has already reached perpendicular and begun swinging around for a 90ish degree turn.
                3.) After the bot is facing the wall head-on, it turns 90ish degrees with 50:50 odds of going clockwise
                    or counterclockwise. NOTE: This is not real path-finding, just a helloworld-type exercise in
                    navigation and ROS architecture more generally.

        :param bot: Instance of ZigZagBot
        :param cmd_pub: rospy.Publisher on topic /cmd_vel_mux/input/navi
        :return:
        """
        move_cmd = Twist()

        if bot.is_bumping:
            # need to face wall before turning 90ish
            # also need to make sure 90ish degree turn NOT already initiated
            # 1/100 of a degree is used as epsilon for perpendicular test
            if not is_perpendicular(bot.left_scan, bot.right_scan, 0.01) and not bot.is_making_90ish_turn:
                # rospy.loginfo("turning toward wall")
                face_wall_direction = 1
                if bot.left_scan - bot.right_scan < 0:  # negative == clockwise, positive  == counterclockwise
                    face_wall_direction = -1
                move_cmd.angular.z = face_wall_direction * 0.1
            else:  # if bot is perpendicular to wall, then it is time to make 90ish degree turn
                if not bot.is_making_90ish_turn:  # if turn just beginning, init turn direction and target orientation
                    # rospy.logwarn("bot  orientation at time of 90ish turn target setting: {}".format(bot.orientation))
                    bot.is_making_90ish_turn = True

                    # turn 90ish degrees left or right with 50:50 odds.
                    turn_90ish_direction = random.choice([-1, 1])  # -1 is clockwise, 1 is counterclockwise

                    # target orientation is actually set at slightly more than 90 degrees to minimize cases of bot
                    # immediately running back into the same wall -- this is necessary because the accuracies of both
                    # the sensed orientations and the issued commands are not very high
                    target_orientation = bot.orientation + (turn_90ish_direction * random.randint(90, 93))

                    # degrees stretch from -180 to 180
                    # 1 -> -1 wraparound is at pi/2 on the unit circle, 179 -> -179 wraparound is at 3pi/2
                    if target_orientation > 180:
                        target_orientation %= -180
                    if target_orientation < -180:
                        target_orientation %= 180

                    bot.target_orientation = target_orientation
                    bot.turn_90ish_direction = turn_90ish_direction
                else:
                    # check if current bot orientation is within a degree of target orientation
                    if abs(bot.orientation - bot.target_orientation) > 1:  # if 90ish turn incomplete, continue
                        # rospy.loginfo("turning 90ish")
                        move_cmd.angular.z = bot.turn_90ish_direction * 0.3
                    else:  # now that 90ish turn is complete, bumping condition should be toggled
                        # rospy.loginfo("finished 90ish turn")
                        bot.is_bumping = False
                        bot.is_making_90ish_turn = False
        else:
            # rospy.loginfo("moving forward")
            move_cmd.linear.x = 0.6  # move forward at 0.6 m/s

        cmd_pub.publish(move_cmd)

    @staticmethod
    def wander_maze_with_bot(bot, victory_grid):
        """Issues wandering movement commands to bot until it fulfills its victory condition.

        :param bot: Instance of ZigZagBot.
        :param victory_grid: A 4-tuple consisting of (x0, x1, y0, y2) that describes the grid exterior.
               Note: Starts top left relative to the origin.
        :return:
        """
        cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            MazeWanderer.move_bot(bot, cmd_vel_pub)
            if MazeWanderer.evaluate_victory(bot, victory_grid):
                rospy.loginfo("Ziggy the ZigZagBot found the box!")
                rospy.loginfo("MazeWanderer exiting...")
                rospy.signal_shutdown("MazeWanderer victory condition reached")

            # rospy.loginfo("bot.left_scan: {}".format(bot.left_scan))
            # rospy.loginfo("bot.right_scan: {}".format(bot.right_scan))
            # rospy.loginfo("bot.is_bumping: {}".format(bot.is_bumping))
            # rospy.loginfo("bot.turn_90ish_direction: {}".format(bot.turn_90ish_direction))
            # rospy.loginfo("bot.orientation: {}".format(bot.orientation))
            # rospy.loginfo("bot.target orientation: {}".format(bot.target_orientation))

            r.sleep()

    @staticmethod
    def evaluate_victory(bot, victory_grid):
        """Checks to see if bot coordinates are within victory grid coordinates.

        :param bot: Instance of ZigZagBot
        :param victory_grid: A 4-tuple consisting of (x0, x1, y0, y2) that describes the grid exterior.
               Note: Starts top left relative to the origin.
        :return:
        """
        return victory_grid[0] < bot.x < victory_grid[1] and victory_grid[2] < bot.y < victory_grid[3]


if __name__ == '__main__':
    try:
        ziggy = ZigZagBot()
        # Hacky hard-coding here, but implementing a smart victory condition was not the real task :p
        grid = (-4.75, -3.25, -9.75, -8.25)
        mw = MazeWanderer(ziggy, grid)
    except rospy.ROSInterruptException:
        pass
