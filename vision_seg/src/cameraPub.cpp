#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

class CameraPublisher {
public:
    CameraPublisher() {
        image_pub = nh.advertise<sensor_msgs::CompressedImage>("camera/arm/compressed", 1);
        capture.open("/dev/video2", cv::CAP_V4L2);
        capture.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        capture.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        capture.set(cv::CAP_PROP_BUFFERSIZE, 1);
    }

    void publishImage() {
        while (ros::ok()) {
            cv::Mat frame;
            capture >> frame;
            if (!frame.empty()) {
                // 이미지를 JPEG 형식으로 압축
                std::vector<uchar> compressed_image;
                cv::imencode(".jpg", frame, compressed_image);
                
                // 압축된 이미지를 CompressedImage 메시지로 변환
                sensor_msgs::CompressedImage compressed_image_msg;
                compressed_image_msg.header.stamp = ros::Time::now();
                compressed_image_msg.format = "jpeg";  // 압축 형식을 지정 (jpeg 또는 png)
                compressed_image_msg.data = compressed_image;

                // 압축된 이미지를 퍼블리시
                image_pub.publish(compressed_image_msg);
            }
        }
    }
    
private:
    ros::NodeHandle nh;
    ros::Publisher image_pub;
    cv::VideoCapture capture;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_arm_publisher");
    CameraPublisher camera_publisher;
    camera_publisher.publishImage();
    return 0;
}
