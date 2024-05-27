#!/usr/bin/env python
# -*- coding:utf-8 -*-
import math, time, rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from pymycobot.mycobot import MyCobot

from cobot_moveit.srv import cali_service_msg, grip_service_msg 

reword_list = [0, 0, 0, 0, 0, 0]  # 로봇암 이동 시, 전 값과의 오차를 저장하기 위한 리스트
state = True
mc = None
data_list = None
def proportional_control() :
    global mc, data_list, state
    tolerance = 0.5
    new_angles = list(data_list)  # 마지막으로 수신한 joint_state 값을 저장
    
    while True :
        comp_list = mc.get_angles()  # 실제 로봇암의 위치를 수신
        print("get_value", comp_list)
        
        errors = [abs(comp_list[i] - data_list[i]) for i in range(6)]   #실제 로봇암의 값과 joint_state의 차이를 게산
        if all(error <= tolerance for error in errors) :   #모든 값이 오차범위 이내라면 탈출
            print("proportional complete!")
            break
        
        for i in range(6) :    #오차범위 이상일 시, 오차의 절반 보정
            if errors[i] > tolerance :
                correction = (data_list[i] - comp_list[i]) / 2
                new_angles[i] += correction
        
        mc.send_angles(new_angles, 40)
        time.sleep(0.1)
       
def callback(data):
    global state, mc, data_list, reword_list
    
    if state and data is not None:
        tmp_list = [0,0,0,0,0,0]
        data_list = []
        for index, value in enumerate(data.position):         #라디안으로 수신한 값을, angle로 변경.
            radians_to_angles = round(math.degrees(value), 2)
            data_list.append(radians_to_angles)
        print("data_list", data_list)
        for i in range(6):
            tmp_list[i] = data_list[i] + reword_list[i]     #직전값에서 생성한 보상값을 더함.
        print("sub_angles", tmp_list)
        mc.send_angles(tmp_list, 40)                        #수정된 리스트를 로봇암에 전송
        time.sleep(0.03)
        get_angle_list = mc.get_angles()
        print("get_angles", get_angle_list)
        for i in range(6):
            reword_list[i] = 0  # 각 데이터 샘플마다 reword_list 초기화
            reword_list[i] = round((tmp_list[i] - get_angle_list[i]) * 0.5,3)     #오차의 절반을 리워드로 설정.
        
def state_callback(data):
    global state
    if data.data == 1:
        state = True
        print("move stop")

def cali_callback(data):
    if data.signal == 1 :
        print("i receive cali_service")
        global state
        state = False
        proportional_control()
    return True
    
def gripper_callback(data):
    print("i receive gripper_service")
    mc.set_gripper_mode(0)
    mc.set_eletric_gripper(0)
    mc.set_gripper_value(data.value,50)
    
    while True:
        gripper_value = mc.get_gripper_value()
        if gripper_value == data:
            break
    rospy.loginfo("gripper value is %d", data.value)
    return True

def listener():
    global mc
    port = rospy.get_param("~port", "/dev/ttyACM0")
    baud = rospy.get_param("~baud", 115200)
    print(port, baud)
    mc = MyCobot(port, baud)
    rospy.init_node("control_slider", anonymous=True)
    
    #moveit의 jointstate를 통해 모터를 제어
    rospy.Subscriber("joint_states", JointState, callback)
    
    #moveit에서의 이동 완료 시 수신, moveit과의 연결을 끊음.
    rospy.Subscriber("state_check", Int32 , state_callback)
    
    #수신하는 joint_state와 실제 로봇암을 일치시켜주기 위한 서비스
    rospy.Service("calibrate_service", cali_service_msg , cali_callback)
    
    #그리퍼의 동작값을 수신하고, 완료를 리턴하는 서비스
    rospy.Service("gripper_service", grip_service_msg , gripper_callback)
    
    time.sleep(0.02)
    mc.set_fresh_mode(1)
    time.sleep(0.03)
    print("spin ...")
    rospy.spin()

if __name__ == "__main__":
    listener()