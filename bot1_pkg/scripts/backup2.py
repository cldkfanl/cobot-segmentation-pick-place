import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion
import actionlib
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from actionlib_msgs.msg import GoalStatus
class MoveToGoal:
    def __init__(self):
        self.goalReached = False
        rospy.init_node('MoveToGoal', anonymous=False)
        self.pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        self.sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_pose_callback)
        self.user_request_sub = rospy.Subscriber('user_request', String, self.user_request_callback)
        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.ac.wait_for_server()
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
        self.pub.publish(sp)
    def amcl_pose_callback(self, msg):
        # You can use this callback to monitor the robot's position if needed
        pass
    def setGoal(self, x, y, z, qx, qy, qz, qw):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position = Point(x, y, z)
        goal.target_pose.pose.orientation = Quaternion(qx, qy, qz, qw)
        self.ac.send_goal(goal)
        rospy.loginfo("Goal sent to action server")
        self.ac.wait_for_result()
        if self.ac.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal reached successfully")
            return True
        else:
            rospy.logwarn("Failed to reach goal")
            return False
    def user_request_callback(self, msg):
        if msg.data == 'dressroom':
            success = self.setGoal(**self.dressroom)
            if success:
                rospy.loginfo("Reached dressroom")
            else:
                rospy.logwarn("Failed to reach dressroom")
        elif msg.data == 'livingroom':
            success = self.setGoal(**self.livingroom)
            if success:
                rospy.loginfo("Reached livingroom")
            else:
                rospy.logwarn("Failed to reach livingroom")
        elif msg.data == 'bathroom':
            success = self.setGoal(**self.bathroom)
            if success:
                rospy.loginfo("Reached bathroom")
            else:
                rospy.logwarn("Failed to reach bathroom")
        else:
            rospy.logwarn("Invalid input")
if __name__ == '__main__':
    try:
        des = MoveToGoal()
        des.initpose()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")