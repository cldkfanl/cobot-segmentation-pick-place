#!/usr/bin/env python

import rospy, cv2
import numpy as np
from vision_seg.msg import *
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from std_msgs.msg import Int32, String
from ultralytics import YOLO
from vision_seg.srv import *

model = YOLO("/home/choi/catkin_ws/src/dressme/vision_seg/src/best.pt")

class Segment(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.camera_matrix = np.array([[542.93802581, 0, 329.25053673], [0, 541.67327024, 256.79448482], [0, 0, 1]])  # 카메라 행렬로 설정
        self.dist_coeffs = np.array([[0.19266232, -0.79141779, -0.00253703, 0.00613584, 1.04252319]])  # 왜곡 계수로 설정
        self.seg_xy_publisher = rospy.Publisher('/seg_cam_xy', seg_center, queue_size=10)
        self.image_sub = None
        self.camera_mode = 'xy_mode'
        self.image_buffer = None
        self.find_seg_class_name = None

    def image_callback(self, msg):
        global model
        print(self.find_seg_class_name)
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv2_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        H, W, _ = cv2_img.shape
        undistorted_img = cv2.undistort(cv2_img, self.camera_matrix, self.dist_coeffs)
        results = model(undistorted_img)
        
        if results:
            for result in results:
                if result.masks:  # 이 조건을 확인
                    for j, mask in enumerate(result.masks.data) :
                        mask = mask.cpu().numpy()
                        class_id = int(result.boxes.cls[j])
                        confidences = result.boxes.conf[j]
                        cls_name = model.names[class_id]
                        if confidences > 0.7 :
                            rgba_img = np.zeros((H, W, 4), dtype=np.uint8)
                        
                            # 마스크 리사이징과 투명도 적용
                            mask_resized = cv2.resize(mask, (W, H))
                            mask_3d = (mask_resized > 0.5).astype(np.uint8)  # 마스크 임계값 조정
                        
                            # 객체의 픽셀만 추출
                            for c in range(3):  # RGB 채널 복사
                                rgba_img[:, :, c] = undistorted_img[:, :, c] * mask_3d

                            rgba_img[:, :, 3] = mask_3d * 255  # 알파 채널 적용(투명도)
                            self.image_buffer = rgba_img
                            
                            if self.camera_mode == 'xy_pub_mode' and self.find_seg_class_name == cls_name :
                                center_x = int((result.boxes.xyxy[j][0] + result.boxes.xyxy[j][2])/2)
                                center_y = int((result.boxes.xyxy[j][1] + result.boxes.xyxy[j][3])/2)
                                print(f"Object class: {cls_name}, Center: ({center_x}, {center_y}), Confidence: {confidences}")
                                
                                if center_x >= 100 and center_x <= 400 :
                                    seg_msg = seg_center()
                                    seg_msg.names = self.find_seg_class_name
                                    seg_msg.x_points = center_x
                                    seg_msg.y_points = center_y
                                    self.seg_xy_publisher.publish(seg_msg)
                        
                            elif self.camera_mode == 'seg_pub_mode' and self.image_buffer is not None :
                                # rgba_img를 압축된 이미지로 변환
                                _, compressed_img = cv2.imencode('.jpg', self.image_buffer)
                                compressed_img_msg = CompressedImage()
                                compressed_img_msg.header.stamp = rospy.Time.now()
                                compressed_img_msg.format = "jpeg"
                                compressed_img_msg.data = np.array(compressed_img).tostring()
                                self.image_buffer = None
                                
                                rospy.wait_for_service('seg_image_service')
                                try:
                                    seg_image_service = rospy.ServiceProxy('seg_image_service', SegImageService)
                                    segrequest = SegImageServiceRequest()
                                    segrequest.image = compressed_img_msg
                                    segrequest.class_name = cls_name
                                    response = seg_image_service(segrequest)
                                    if response.success:
                                        print("이미지 수락됨, xy_mode로 전환")
                                    else:
                                        print("이미지가 수락되지 않음, 재시도 중...")
                                    self.camera_mode = 'xy_mode'
                                except rospy.ServiceException as e:
                                    print("서비스 호출 실패: %s" % e)
                
    def control_image_subscription(self, state):
        if state and self.image_sub is None:
            self.image_sub = rospy.Subscriber("/camera/arm/compressed", CompressedImage, self.image_callback)
        elif not state and self.image_sub is not None:
            self.image_sub.unregister()
            self.image_sub = None
            
def capture_callback(msg, segment_img):
    if msg.data == 1 :
        print("i subscribed for capture")
        segment_img.camera_mode = 'seg_pub_mode'
        
def callback(msg, segment_img):
    if msg.data is not None :
        rospy.loginfo("Starting img segment")
        segment_img.find_seg_class_name = msg.data
        segment_img.camera_mode = 'xy_pub_mode'
        segment_img.control_image_subscription(True)

def segment_server(segment_img):
    rospy.init_node('yolo_segment_server')
    rospy.Subscriber("seg_start", String, callback, callback_args=segment_img)
    rospy.Subscriber("capture_start_topic", Int32, capture_callback, callback_args=segment_img)
    rospy.spin()

def main():
    segment_img = Segment()
    segment_server(segment_img)

if __name__ == "__main__":
    main()
