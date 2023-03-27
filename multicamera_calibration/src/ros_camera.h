#ifndef ROS_CAMERA_H
#define ROS_CAMERA_H

#include <string>

#include "ros/ros.h"

#include <sstream>
#include <iostream>
#include <string>

#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <chrono>

#include <atomic>
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <Eigen/Geometry>

struct BoardConfiguration{
    std::vector<std::vector<cv::Point3f>> objPoints;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    std::vector<int> ids;
};

struct BoardCameraExtrinsic{
        cv::Vec4d quaternion; 
        cv::Vec3d tvec;
        uint camIndex;
        ros::Time stamp;
};

class RosCamera{
	public:
		RosCamera(ros::NodeHandle nh, std::string topicName, unsigned int cameraIndex, BoardConfiguration board, cv::Mat intrinsic);
		~RosCamera();
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
    	// ros::Subscriber cam_info_sub_;          // Subscriber
    	image_transport::Subscriber imageSub_; // Subscriber

		bool arucoBoardDetection(cv::Mat &image, cv::Vec4d &quaternion, cv::Vec3d &tvec);

		void cameraImageCallback(const sensor_msgs::ImageConstPtr &msg);

		
		std::vector<BoardCameraExtrinsic> boardExtrinsics;


		cv::Mat rvectvecToTransformationMatrix(cv::Vec3d &rvec, cv::Vec3d &tvec);
		cv::Vec4d rvecToQuaternion(cv::Vec3d rvec);

		void imageQueueProcessing();
		std::atomic<bool> runThread_;
		std::queue<cv::Mat> imageQueue;
		std::queue<ros::Time> timeStampQueue;
		std::mutex mut;
		std::condition_variable cv;
		std::thread thr;

};

#endif // ROS_CAMERA_H