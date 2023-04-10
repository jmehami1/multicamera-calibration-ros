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

/**
 * @brief Stores board parameters required for pose estimation
 * 
 */
struct BoardConfiguration{
    std::vector<std::vector<cv::Point3f>> objPoints;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    std::vector<int> ids;
};

/**
 * @brief Stores estimated extrinsic poses of a single board with the camera ID and time information
 * 
 */
struct BoardCameraExtrinsic{
        cv::Vec4d quaternion; 
        cv::Vec3d tvec;
        uint camIndex;
        ros::Time stamp;
};

/*!
 *  \brief	ROS camera class and board pose estimation
 *  \details
 *  This is the ROS camera class which subscribes to a given list of image topics which publish images of the double-sded aruco board.
 * 	Each of the subscribed camera topics must provide the camera intrinsic parameters via the file camera_info.yaml
 *  \author    Jasprabhjit Mehami
 *  \version   1.0
 *  \date      2023-04-10
 *  \pre       none
 *  \bug       none reported as of 2023-04-10
 *  \warning   
 */

class RosCamera{
	public:
		/**
		 * @brief Construct a new Ros Camera object
		 * 
		 * @param nh 
		 * @param topicName 
		 * @param cameraIndex 
		 * @param board 
		 * @param intrinsic 
		 */
		RosCamera(ros::NodeHandle nh, std::string topicName, unsigned int cameraIndex, BoardConfiguration board, cv::Mat intrinsic);

		/**
		 * @brief Get a copy of the estimated board poses
		 * 
		 * @return std::vector<BoardCameraExtrinsic> 
		 */
		std::vector<BoardCameraExtrinsic> getCopyBoardExtrinsicData();

		/**
		 * @brief Get the last image message time
		 * 
		 * @return std::chrono::time_point<std::chrono::high_resolution_clock> 
		 */
		std::chrono::time_point<std::chrono::high_resolution_clock> getLastMsgTime();

		/**
		 * @brief Get the state of the first message flag
		 * 
		 * @return true 
		 * @return false 
		 */
		bool getFirstMsgFlag();
		
	private:
	/**
	 * @brief 
	 * 
	 */
		std::string topicName_;	/**< Subscriber topic name */
		BoardConfiguration board_; /**< The current board configuration parameters required for estimating poses*/
		cv::Ptr<cv::aruco::DetectorParameters> parameters_; /**< Board detector parameters */
		cv::Ptr<cv::aruco::Board> arucoBoard_; /**< ArUco board object*/
		cv::Mat intrinsic_; /**< Intrinsic parameters of the camera */
		cv::Mat distCoeffs_; /**< Distortion coefficient of the camera in OpenCV format */

		ros::NodeHandle nh_; /**< Node handler object */
		unsigned int cameraIndex_; /**< Index number given to image topic */
    	image_transport::ImageTransport it_; /**< Image transporter object */
    	image_transport::Subscriber imageSub_; /**< Image subscriber */

		std::vector<BoardCameraExtrinsic> boardExtrinsics; /**< Stores the poses that are estimated from image message */
		std::chrono::time_point<std::chrono::high_resolution_clock> lastMsgTime_; /**< Last message time taken from ROS timestamp */
		bool firstMsgReceivedFlag_; /**< First message received flag */

		/**
		 * @brief Attempts to detect an ArUco board, and if found, estimates the extrinsic pose
		 * 
		 * @param image 
		 * @param quaternion 
		 * @param tvec 
		 * @return true 
		 * @return false 
		 */

		bool arucoBoardExtrinsic(cv::Mat &image, cv::Vec4d &quaternion, cv::Vec3d &tvec);
		/**
		 * @brief ROS image callback function
		 * 
		 * @param msg 
		 */
		void cameraImageCallback(const sensor_msgs::ImageConstPtr &msg);

		/**
		 * @brief Converts a Rodrigues axis-angle to a quaternion
		 * 
		 * @param rvec 
		 * @return cv::Vec4d 
		 */
		cv::Vec4d rvecToQuaternion(cv::Vec3d rvec);

};

#endif // ROS_CAMERA_H