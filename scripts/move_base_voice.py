#!/usr/bin/env python
import rospy
import math
import collections
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import roslib

roslib.load_manifest('move_base')
import actionlib
import tf

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

goalQueue = collections.deque()
numwords = {}

# based on voice_cmd_vel
class move_base_voice:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        rospy.Subscriber('recognizer/output', String, self.speechCb)

    def speechCb(self, msg):
        rospy.loginfo(msg.data)

        splitreq = msg.data.split()

        if len(splitreq) == 0:
            return

        if splitreq[0] == "move":
            if len(splitreq) < 2:
                return

            has_direction = splitreq[1] == "backward" or splitreq[1] == "forward"
            negate = splitreq[1] == "backward"
            dist_meters = 3

            if (not has_direction) or len(splitreq) > 2:
                slice_start = 2 if has_direction else 1

                dist, unitIndex = move_base_voice.try_parse_number(splitreq[slice_start:])

                if unitIndex >= len(splitreq):
                    unit = "meters"
                else:
                    unit = splitreq[unitIndex]

                if unit == "meter" or unit == "meters":
                    dist_meters = dist
                elif unit == "foot" or unit == "feet":
                    dist_meters = dist * 0.3048
                elif unit == "inch" or unit == "inches":
                    dist_meters = (dist / 12.0) * 0.3048
                else:
                    dist_meters = dist

            if negate:
                self.speechCb(String("turn left one hundred eighty degrees"))
            g = MoveBaseGoal()
            g.target_pose.pose.position.x = dist_meters
            goalQueue.append(g)

        elif splitreq[0] == "turn":
            if len(splitreq) < 2:
                return

            has_direction = splitreq[1] == "right" or splitreq[1] == "left"
            negate = splitreq[1] == "right"
            angle_rad = math.pi / 2.0

            if (not has_direction) or len(splitreq) > 2:
                slice_start = 2 if has_direction else 1

                angle, unitIndex = move_base_voice.try_parse_number(splitreq[slice_start:])

                if unitIndex >= len(splitreq):
                    unit = "degrees"
                else: unit = splitreq[unitIndex]

                if unit == "degree" or unit == "degrees":
                    angle_rad = math.radians(angle)
                else: angle_rad = angle

            q = tf.transformations.quaternion_from_euler(0, 0, angle_rad * (-1 if negate else 1))
            g = MoveBaseGoal()
            g.target_pose.pose.orientation.x = q[0]
            g.target_pose.pose.orientation.y = q[1]
            g.target_pose.pose.orientation.z = q[2]
            g.target_pose.pose.orientation.w = q[3]
            goalQueue.append(g)
        elif splitreq[0] == "dance":
            for i in xrange(0, 4):
                self.speechCb(String("turn left ninety degrees"))
                self.speechCb(String("turn right ninety degrees"))
                self.speechCb(String("turn right ninety degrees"))
                self.speechCb(String("turn left ninety degrees"))

                self.speechCb(String("move backward"))
                self.speechCb(String("move forward"))

                self.speechCb(String("turn left ninety degrees"))
        elif splitreq[0] == "stop":
            # immediately clear the queue so everything stops executing
            goalQueue.clear()
        elif splitreq[0] == "spin":
            for i in xrange(0, 100):
                self.speechCb(String("turn left three hundred sixty degrees"))

    def cleanup(self):
        goalQueue.clear()

    @staticmethod
    def try_parse_number(nums):
        """
        Tries to take a list of words and turn it into an actual number.
        Drawn from https://stackoverflow.com/a/493788
        :param nums: List of number words
        :return: A numerical type represented by the list, and the index in the list of the first non-number word
        """

        if len(numwords) == 0:
            units = [
                "zero", "one", "two", "three", "four", "five", "six", "seven", "eight", "nine", "ten",
                "eleven", "twelve", "thirteen", "fourteen", "fifteen", "sixteen", "seventeen", "eighteen", "nineteen",
                "twenty"
            ]
            tens = ["", "", "twenty", "thirty", "forty", "fifty", "sixty", "seventy", "eighty", "ninety"]
            scales = ["hundred", "thousand", "million", "billion", "trillion"]
            numwords["and"] = (1, 0)
            for idx, word in enumerate(units): numwords[word] = (1, idx)
            for idx, word in enumerate(tens): numwords[word] = (1, idx * 10)
            for idx, word in enumerate(scales): numwords[word] = (10 ** (idx * 3 or 2), 0)

        slice_target = len(nums)
        for i in xrange(0, len(nums)):
            if not nums[i] in numwords:
                slice_target = i

        slice = nums[:slice_target]

        result = current = 0
        for word in slice:
            scale, increment = numwords[word]
            current = current * scale + increment
            if scale > 100:
                result += current
                current = 0

        return result + current, slice_target

if __name__ == '__main__':
    rospy.init_node('move_base_voice')
    try:
        move_base_voice()
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        # client.wait_for_server()
        while not rospy.is_shutdown() and len(goalQueue) != 0:
            try:
                g = goalQueue.popleft()
                client.send_goal(g)
                client.wait_for_result(rospy.Duration.from_sec(2.0))
            except IndexError:
                pass
    except rospy.ROSInterruptException:
        pass
