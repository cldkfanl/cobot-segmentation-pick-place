import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion
import actionlib
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String

class MoveToGoal:
    def __init__(self):
        self.goalReached = False
        rospy.init_node('MoveToGoal', anonymous=False)
        self.pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        self.sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped)
        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.msub = rospy.Subscriber('user_request', String)
        self.ac.wait_for_server()
    
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
if __name__ == '__main__':
    try:
        des = MoveToGoal()
        des.initpose()
        # 방의 좌표 및 방향 설정
        room1 = {
            "x": 3.4885525953642715,
            "y": -0.7352970983316875,
            "z": 0.0,
            "qx": 0.0,
            "qy": 0.0,
            "qz": -0.7294276133064245,
            "qw": 0.6840580070038602
        }
        room2 = {
            "x": 4.608703518711924,
            "y": -4.279103430373378,
            "z": 0.0,
            "qx": 0.0,
            "qy": 0.0,
            "qz": 0.010387099614432852,
            "qw": 0.9999460526256403
        }
        room3 = {
            "x": 4.7729290673860705,
            "y": -0.44602149182562295,
            "z": 0.0,
            "qx": 0.0,
            "qy": 0.0,
            "qz": 0.7215434603619519,
            "qw": 0.6923691463438418
        }
        # 입력 대기 및 이동 명령 처리
        while True:
            user_input = input("Type room number (room1, room2, room3) to move or 'exit' to quit: ")
            if user_input == 'exit':
                break
            elif user_input == 'room1':
                success = des.setGoal(**room1)
                if success:
                    rospy.loginfo("Reached room1")
                else:
                    rospy.logwarn("Failed to reach room1")
            elif user_input == 'room2':
                success = des.setGoal(**room2)
                if success:
                    rospy.loginfo("Reached room2")
                else:
                    rospy.logwarn("Failed to reach room2")
            elif user_input == 'room3':
                success = des.setGoal(**room3)
                if success:
                    rospy.loginfo("Reached room3")
                else:
                    rospy.logwarn("Failed to reach room3")
            else:
                rospy.logwarn("Invalid input")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")