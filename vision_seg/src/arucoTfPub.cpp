#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/CompressedImage.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <vision_seg/aruco_center.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Dense>

class Aruco {
public:
    Aruco() : MARKERLEN(0.036){
        DICT_GET = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);  // 
        ARUCO_PARAMETERS = cv::aruco::DetectorParameters::create();
        
        double camera_matrix_data[3][3] = {{542.93802581, 0, 329.25053673}, {0, 541.67327024, 256.79448482}, {0, 0, 1}};  // 카메라 행렬로 설정
        camera_matrix = cv::Mat(3, 3, CV_64F, camera_matrix_data).clone();
        
        double dist_coeffs_data[1][5] = {{0.19266232, -0.79141779, -0.00253703, 0.00613584, 1.04252319}};  // 왜곡 계수로 설정
        dist_coeffs = cv::Mat(1, 5, CV_64F, dist_coeffs_data).clone();

        tf_broadcaster = std::make_shared<tf::TransformBroadcaster>();
        aruco_xy_publisher = nh.advertise<vision_seg::aruco_center>("aruco_cam_xy", 10);
    }

    void startServer() {
        image_sub = nh.subscribe("camera/arm/compressed", 1, &Aruco::image_callback, this);
        aruco_start = nh.subscribe("aruco_start", 1, &Aruco::aruco_callback, this);
        ros::spin();
    }

    void matrix_to_transform(const cv::Vec3d& rvec, const cv::Vec3d& tvec, geometry_msgs::TransformStamped& transform_stamped) {
        cv::Mat R;
        cv::Rodrigues(rvec, R);

        Eigen::Matrix3d rotation;
        rotation << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
                    R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
                    R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2);

        Eigen::Quaterniond quaternion(rotation);
        Eigen::Vector3d translation(tvec[0], tvec[1], tvec[2]);

        transform_stamped.transform.translation.x = translation.x();
        transform_stamped.transform.translation.y = translation.y();
        transform_stamped.transform.translation.z = translation.z();

        transform_stamped.transform.rotation.x = quaternion.x();
        transform_stamped.transform.rotation.y = quaternion.y();
        transform_stamped.transform.rotation.z = quaternion.z();
        transform_stamped.transform.rotation.w = quaternion.w();
    }

    void trigger_callback(const std_msgs::Int32::ConstPtr& msg) {
        arucoStart2 = true;
        findArucoID = msg->data;
    }

    void aruco_callback(const std_msgs::Int32::ConstPtr& msg) {
        if(msg->data == 1) {
            ROS_INFO("Aruco Start");
            image_state = true;
        }
        else{
            ROS_INFO("Aruco Stop");
            image_state = false;
        }
    }
    void image_callback(const sensor_msgs::CompressedImageConstPtr& msg) {
        cv::Mat cv2_img = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        std::vector<std::vector<cv::Point2f>> markerCorners;
        std::vector<int> markerIds;
        std::vector<cv::Vec3d> rvecs, tvecs;

        cv::aruco::detectMarkers(cv2_img, DICT_GET, markerCorners, markerIds, ARUCO_PARAMETERS);

        if (!markerIds.empty() && image_state) {
            vision_seg::aruco_center aruco_msg;
            cv::aruco::estimatePoseSingleMarkers(markerCorners, MARKERLEN, camera_matrix, dist_coeffs, rvecs, tvecs);                
            
            for (size_t i = 0; i < markerIds.size(); ++i) {
                geometry_msgs::TransformStamped transform_stamped;

                matrix_to_transform(rvecs[i], tvecs[i], transform_stamped);

                aruco_msg.names.push_back(std::to_string(markerIds[i]));
                aruco_msg.x_points.push_back((markerCorners[i][0].x + markerCorners[i][2].x) / 2);
                aruco_msg.y_points.push_back((markerCorners[i][0].y + markerCorners[i][2].y) / 2);

                std::string frame_id = "marker_" + std::to_string(markerIds[i]);
                transform_stamped.header.stamp = ros::Time::now();
                transform_stamped.header.frame_id = "robotarm/cam_lens_link";
                transform_stamped.child_frame_id = frame_id;
                tf_broadcaster->sendTransform(transform_stamped);

                ROS_INFO("Marker ID: %d", findArucoID);
                ROS_INFO("X: %f", aruco_msg.x_points[i]);
                ROS_INFO("Y: %f", aruco_msg.y_points[i]);
            }
            aruco_xy_publisher.publish(aruco_msg);
        }
    }
private:
    ros::NodeHandle nh;
    ros::Subscriber image_sub;
    ros::Subscriber aruco_start;
    ros::Subscriber test_sub;
    ros::Publisher aruco_xy_publisher;
    ros::Publisher tvecs_publisher = nh.advertise<std_msgs::Float32MultiArray>("aruco_tvecs", 10);
    std::shared_ptr<tf::TransformBroadcaster> tf_broadcaster;
    bool image_state = false;
    bool arucoStart2 = false;
    int findArucoID;
    const float MARKERLEN;
    cv::Ptr<cv::aruco::Dictionary> DICT_GET;
    cv::Ptr<cv::aruco::DetectorParameters> ARUCO_PARAMETERS;
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    std::vector<double> camera_matrix_data;
    std::vector<double> dist_coeffs_data;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "arucoTfPub");
    Aruco aruco ;
    aruco.startServer();
    return 0;
}
