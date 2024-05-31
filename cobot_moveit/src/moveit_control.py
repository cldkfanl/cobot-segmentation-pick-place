#!/usr/bin/env python
import sys, rospy, moveit_commander, time, tf, queue
import geometry_msgs.msg
from moveit_msgs.msg import MoveItErrorCodes
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32, String
from cobot_moveit.srv import *


#옷장 바라보는 방향
pose_closet_orientation = ({
    'x' : -0.7071080795300609,
    'y' : 0,
    'z' : 0.7071080795300609,
    'w' : 0
})
#사람 바라보는 방향
pose_person_orientation = ({
    'x' : 0,
    'y' : 0.7071080795300609,
    'z' : 0,
    'w' : 0.7071080795300609
})
#초기상태
pose_basic_orientation = ({
    'x' : 0.0,
    'y' : 0.7071080795300609,
    'z' : -0.7071080795300609,
    'w' : 0.0
})
pose_find_closet_position = ({
    'x' : 0.0,
    'y' : -0.08,
    'z' : 0.4
})
pose_put_cloth_position = ({
    'x' : 0.25,
    'y' : -0.08,
    'z' : 0.45
})
pose_basic_position = ({
    'x' : 0.0,
    'y' : -0.292,
    'z' : 0.523
})

aruco_start_pub = None
seg_start_pub = None
move_start_pub = None
gripper_service = None
start_state = False
hanger_position = queue.Queue()
second_put_mode = None

def cali_service_start() :
    rospy.wait_for_service('calibrate_service')
    try :
        calibrate_service = rospy.ServiceProxy('calibrate_service', basic_service)
        req = basic_serviceRequest()
        req.value = 1
        calibrate_service(req)
        
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def recent_tf_service_start(classname) :
    rospy.wait_for_service('recent_tf_service')
    try :
        recent_tf_service = rospy.ServiceProxy('recent_tf_service', find_tf_service)
        req = find_tf_serviceRequest()
        req.class_name = classname
        response = recent_tf_service(req)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
    return response
        
def move_to_pose(pose, ori):
    state_check_pub.publish(1)
    
    time.sleep(1)
    moveit_commander.roscpp_initialize(sys.argv)

    robot = moveit_commander.RobotCommander()

    arm_group = moveit_commander.MoveGroupCommander("arm_group")
    arm_group.set_planner_id("RRTConnectkConfigDefault")
    # 계획 시도 횟수 설정
    arm_group.set_num_planning_attempts(30)
    
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position = geometry_msgs.msg.Point(**pose)
    pose_goal.orientation = geometry_msgs.msg.Quaternion(**ori)

    arm_group.set_pose_target(pose_goal)
    plan = arm_group.go(wait=True)  # 이동이 완료될 때까지 대기
    if plan:
        print("Movement successful!")
    else:
        print("Movement failed!")
    arm_group.stop()
    arm_group.clear_pose_targets()
    time.sleep(1)

def move_cobot_and_calib(xyz, ori) :
    move_to_pose(xyz, ori)
    cali_service_start()
    print("i return cali_service")

def gripper_move(val) :
    global gripper_service
    rospy.wait_for_service('gripper_service')
    try :
        gripper_service = rospy.ServiceProxy('gripper_service', basic_service)
        gripper_service(val)
        print("i return gripper_service")
        
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
    
def get_tf_position(name, x, y, z) :
    listener = tf.TransformListener()
    marker_frame = name
    pose = None
    listener.waitForTransform("base", marker_frame, rospy.Time(0), rospy.Duration(4.0))
    if listener.canTransform("base", marker_frame, rospy.Time(0)):
        (trans, rot) = listener.lookupTransform("base", marker_frame, rospy.Time(0))
        pose = trans[0], trans[1], trans[2]
    tmp_pose = {
        'x': pose[0] + x,
        'y': pose[1] + y,
        'z': pose[2] - z
    }
    return tmp_pose

def third_callback(request) :
    global hanger_position
    
    hanger_pose = hanger_position.get()
    gripper_move(10)
    
    tmp_pose = {
        'x': hanger_pose['x'] + 0.05,
        'y': hanger_pose['y'],
        'z': hanger_pose['z']
    }
    
    move_to_pose(tmp_pose, pose_closet_orientation)
    
    time.sleep(1)
    
    move_cobot_and_calib(hanger_pose, pose_closet_orientation)
    
    gripper_move(75)
    move_to_pose(tmp_pose, pose_closet_orientation)
    
    time.sleep(1)
    
    move_cobot_and_calib(pose_find_closet_position, pose_closet_orientation)
    
    hanger_position.queue.clear()
    
    rospy.loginfo("Returning success: %s (type: %s)", True, type(True))        
    return basic_serviceResponse(True)  # 수신완료 리턴


def second_callback(request) :
    global hanger_position
    print("i return second_service")
    
    seg_start_pub.publish(request.class_name)
    recent_aruco = recent_tf_service_start(request.class_name)
    
    find_pose = get_tf_position(recent_aruco, 0.5, 0, -0.3)
    
    print(find_pose)
    move_cobot_and_calib(find_pose, pose_closet_orientation)
    gripper_move(100)
    
    
    tmp_pose = get_tf_position(recent_aruco, 0, 0, -0.3)
    
    print(tmp_pose)
    
    hanger_position.put(tmp_pose)
    
    gripper_move(10)
    
    move_to_pose(tmp_pose, pose_closet_orientation)
    time.sleep(1)
    move_cobot_and_calib(pose_put_cloth_position, pose_person_orientation)
    gripper_move(75)
    
    rospy.loginfo("Returning success: %s (type: %s)", True, type(True))        
    return second_service_msgResponse(True)  # 수신완료 리턴
    
def robotarm_main_callback(request) :
    print("i receive robotarm_action_service")      #GUI에서 동작 요청 수신 시
    global start_state
    if request.mode == 'start_mode' :       #시작, 카메라 촬영 포지션 이동
        move_cobot_and_calib(pose_find_closet_position, pose_closet_orientation)
        aruco_start_pub.publish(1)      #aruco, seg 시작 명령
        start_state = True
        success = True
    elif request.mode == 'end_mode' :       #종료, 초기 표지션 이동
        move_cobot_and_calib(pose_basic_position, pose_basic_orientation)
        aruco_start_pub.publish(2)      #aruco, seg 종료 명령
        start_state = False
        success = True
    else:
        success = False  # 요청이 잘못된 경우
    
    rospy.loginfo("Returning success: %s (type: %s)", success, type(success))        
    return main_service_msgResponse(success)  # 수신완료 리턴

def main():
    global state_check_pub, start_state, aruco_start_pub, seg_start_pub

    # ROS 노드 초기화
    rospy.init_node('dressme_moveit_node', anonymous=True)
    state_check_pub = rospy.Publisher("state_check", Int32, queue_size=10)
    aruco_start_pub = rospy.Publisher("aruco_start", Int32, queue_size=10)
    seg_start_pub = rospy.Publisher("seg_start", String, queue_size=10)
    
    
    main_server = rospy.Service("robotarm_action_service", main_service_msg, robotarm_main_callback)
    second_service = rospy.Service("second_service", second_service_msg, second_callback)
    third_service = rospy.Service("third_service",basic_service, third_callback)
    
    
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass