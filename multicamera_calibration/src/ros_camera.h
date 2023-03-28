#ifndef ROS_CAMERA_H
#define ROS_CAMERA_H

#include <string>
#include "ros/ros.h"
#include <iostream>
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <Eigen/Geometry>
#include <chrono>

// Saving individual aruco board parameters
struct BoardConfiguration{
    std::vector<std::vector<cv::Point3f>> objPoints;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    std::vector<int> ids;
};

// estimated board extrinsic with timestamp and camera id
struct BoardCameraExtrinsic{
        cv::Vec4d quaternion; 
        cv::Vec3d tvec;
        uint camIndex;
        ros::Time stamp;
};

class RosCamera{
	public:
		RosCamera(ros::NodeHandle nh, std::string topicName, unsigned int cameraIndex, BoardConfiguration board, cv::Mat intrinsic);
		std::vector<BoardCameraExtrinsic> getCopyBoardExtrinsicData();
	private:
		std::string topicName_;
		BoardConfiguration board_;
		cv::Ptr<cv::aruco::DetectorParameters> parameters_;
		cv::Ptr<cv::aruco::Board> arucoBoard_;
		cv::Mat intrinsic_;
		cv::Mat distCoeffs_;

		ros::NodeHandle nh_;
		unsigned int cameraIndex_;
    	image_transport::ImageTransport it_;
    	image_transport::Subscriber imageSub_;

		std::vector<BoardCameraExtrinsic> boardExtrinsics;

		bool arucoBoardExtrinsic(cv::Mat &image, cv::Vec4d &quaternion, cv::Vec3d &tvec);
		void cameraImageCallback(const sensor_msgs::ImageConstPtr &msg);
		cv::Mat rvectvecToTransformationMatrix(cv::Vec3d &rvec, cv::Vec3d &tvec);
		cv::Vec4d rvecToQuaternion(cv::Vec3d rvec);

};

#endif // ROS_CAMERA_H