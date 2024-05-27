import rospy
import sys
import math
import moveit_commander
from moveit_msgs.msg import PlanningScene, AllowedCollisionEntry, AllowedCollisionMatrix

def degrees_to_radians(degrees):
    return [angle * (math.pi / 180.0) for angle in degrees]

def move_to_jointpose():
    # ROS 노드를 초기화합니다.
    rospy.init_node('move_joint_pose_node', anonymous=True)
    print("move_to_jointpose start")
    
    # MoveIt! 초기화
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    
    # 각도를 설정합니다.
    arm_group = moveit_commander.MoveGroupCommander("arm_group")
    angles_in_degrees = [0, -55, 25, 30, -90, 90] # 값 변경
    radian_pose = degrees_to_radians(angles_in_degrees)
    print("Target joint pose in radians:", radian_pose)
    
    # 목표 관절 각도를 설정합니다.
    arm_group.set_joint_value_target(radian_pose)
    
    # 관절 값으로 이동합니다.
    arm_group.go(wait=True)
    
    # 상태를 리셋합니다.
    arm_group.stop()
    arm_group.clear_pose_targets()
    

if __name__ == '__main__':
    try:
        move_to_jointpose()
    except rospy.ROSInterruptException:
        pass
