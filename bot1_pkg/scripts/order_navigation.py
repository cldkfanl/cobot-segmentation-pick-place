#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Point, Quaternion
from actionlib_msgs.msg import GoalID, GoalStatusArray

class MoveToGoal:
    def __init__(self):
        self.goalReached = False
        self.current_goal = None
        self.previous_goal = None
        rospy.init_node('MoveToGoal', anonymous=False)
        self.posepub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
        self.amclsub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_pose_callback)
        self.user_request_sub = rospy.Subscriber('user_request', String, self.user_request_callback)
        self.goal_status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.goal_status_callback)

        self.dressroom = {
            "x": 3.4885525953642715,
            "y": -0.7352970983316875,
            "z": 0.0,
            "qx": 0.0,
            "qy": 0.0,
            "qz": -0.7294276133064245,
            "qw": 0.6840580070038602
        }
        self.livingroom = {
            "x": 4.608703518711924,
            "y": -4.279103430373378,
            "z": 0.0,
            "qx": 0.0,
            "qy": 0.0,
            "qz": 0.010387099614432852,
            "qw": 0.9999460526256403
        }
        self.bathroom = {
            "x": 4.7729290673860705,
            "y": -0.44602149182562295,
            "z": 0.0,
            "qx": 0.0,
            "qy": 0.0,
            "qz": 0.7215434603619519,
            "qw": 0.6923691463438418
        }
        self.startpoint = {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "qx": 0.0,
            "qy": 0.0,
            "qz": 0.0,
            "qw": 1.0
        }

    def initpose(self):
        sp = PoseWithCovarianceStamped()
        sp.header.seq = 0
        sp.header.stamp.secs = 0
        sp.header.stamp.nsecs = 0
        sp.header.frame_id = 'map'
        sp.pose.pose.position.x = 0.0
        sp.pose.pose.position.y = 0.0
        sp.pose.pose.position.z = 0.0
        sp.pose.pose.orientation.x = 0.0
        sp.pose.pose.orientation.y = 0.0
        sp.pose.pose.orientation.z = 0.0
        sp.pose.pose.orientation.w = 1.0
        rospy.sleep(1)
        self.posepub.publish(sp)

    def amcl_pose_callback(self, msg):
        # You can use this callback to monitor the robot's position if needed
        pass

    def setGoal(self, x, y, z, qx, qy, qz, qw):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position = Point(x, y, z)
        goal.pose.orientation = Quaternion(qx, qy, qz, qw)
        self.previous_goal = self.current_goal
        self.current_goal = {"x": x, "y": y, "z": z, "qx": qx, "qy": qy, "qz": qz, "qw": qw}
        self.goal_pub.publish(goal)
        rospy.loginfo("Goal sent to /move_base_simple/goal")

    def user_request_callback(self, msg):
        if msg.data == 'dressroom':
            if self.current_goal != self.dressroom:
                self.setGoal(**self.dressroom)
                rospy.loginfo("Moving to dressroom")
        elif msg.data == 'livingroom':
            if self.current_goal != self.livingroom:
                self.setGoal(**self.livingroom)
                rospy.loginfo("Moving to livingroom")
        elif msg.data == 'bathroom':
            if self.current_goal != self.bathroom:
                self.setGoal(**self.bathroom)
                rospy.loginfo("Moving to bathroom")
        elif msg.data == 'startpoint':
            if self.current_goal != self.startpoint:
                self.setGoal(**self.startpoint)
                rospy.loginfo("Moving to startpoint")
        elif msg.data == 'stop':
            self.cancel_goal()
            rospy.loginfo("Stopping the robot")
        else:
            rospy.logwarn("Invalid input")

    def goal_status_callback(self, msg):
        if not self.current_goal:
            return

        for status in msg.status_list:
            if status.status == 3:  # Status 3 means the goal was reached
                rospy.loginfo("Goal reached")
                self.goalReached = True
                self.on_goal_reached()
                break

    def on_goal_reached(self):
        if self.current_goal == self.dressroom:
            self.arm_pub.publish('start_mode')
            rospy.loginfo("Arrived at dressroom. Arm mode set to start_mode.")
        self.previous_goal = self.current_goal
        self.current_goal = None

    def cancel_goal(self):
        cancel_msg = GoalID()
        self.cancel_pub.publish(cancel_msg)
        rospy.loginfo("Goal cancelled")

if __name__ == '__main__':
    try:
        des = MoveToGoal()
        des.initpose()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")
