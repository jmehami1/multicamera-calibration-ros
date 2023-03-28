#include "ros_camera.h"

RosCamera::RosCamera(ros::NodeHandle nh, std::string topicName, unsigned int cameraIndex, BoardConfiguration board, cv::Mat intrinsic):
nh_(nh), topicName_(topicName), cameraIndex_(cameraIndex), board_(board), intrinsic_(intrinsic), it_(nh)
{
	// Initialise the ArUco board configuration
  	parameters_ = cv::aruco::DetectorParameters::create();
	arucoBoard_ = cv::aruco::Board::create(board.objPoints, board.dictionary, board.ids);

	// subscribe to image topic
	imageSub_ = it_.subscribe(topicName_, 1000, &RosCamera::cameraImageCallback, this);

	ROS_INFO_STREAM("subscribed to " << topicName_);

}

void RosCamera::cameraImageCallback(const sensor_msgs::ImageConstPtr &msg)
{
	// Convert sensor_msgs/Image to cv::Mat
	cv::Mat inputImage = cv_bridge::toCvShare(msg, "bgr8")->image; 

  	cv::Vec4d quaternion; cv::Vec3d tvec;

	//perform pose estimation
	if (arucoBoardExtrinsic(inputImage, quaternion, tvec)){
		auto timeStamp = msg->header.stamp;
		BoardCameraExtrinsic curExt;
		curExt.camIndex = cameraIndex_;
		curExt.quaternion = quaternion;
		curExt.tvec = tvec;
		curExt.stamp = timeStamp;
		boardExtrinsics.push_back(curExt);
	}

}


bool RosCamera::arucoBoardExtrinsic(cv::Mat &image, cv::Vec4d &quaternion, cv::Vec3d &tvec)
{
	quaternion = 0;
    tvec = 0;

	//detect aruco markers from images
	std::vector<int> markerIds; // Create a vector contains marker id
	std::vector<std::vector<cv::Point2f>> markerCorners;
	cv::aruco::detectMarkers(image, board_.dictionary, markerCorners, markerIds, parameters_);

	//check if any markers are found
	if (markerIds.size() < 1)
		return false;

	//get the detected marker image coordinates and their corresponding world coordinates
	std::vector<cv::Point3f> objPoints;
	std::vector<cv::Point2f> imgPoints;
	cv::aruco::getBoardObjectAndImagePoints(arucoBoard_, markerCorners, markerIds, objPoints, imgPoints);

	//check if atleast four corners of one marker are determined
	if (objPoints.size() < 4)
		return false;
	
	//solve PnP to estimate pose of board
	cv::Vec3d rvec;
	cv::solvePnP(objPoints, imgPoints, intrinsic_, distCoeffs_, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
	quaternion = rvecToQuaternion(rvec);

	return true;
}

std::vector<BoardCameraExtrinsic> RosCamera::getCopyBoardExtrinsicData()
{
	return boardExtrinsics;
} 

cv::Mat RosCamera::rvectvecToTransformationMatrix(cv::Vec3d &rvec, cv::Vec3d &tvec)
{
	cv::Mat R;              // Rotation matrix
	cv::Rodrigues(rvec, R); // Convert rvec (1x3) to rotation matrix (3x3)

	cv::Mat tf = (cv::Mat_<double>(4, 4) << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), tvec[0],
					R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), tvec[1],
					R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), tvec[2],
					0, 0, 0, 1);

	return tf;
}

cv::Vec4d RosCamera::rvecToQuaternion(cv::Vec3d rvec)
{
	cv::Mat R;
	cv::Rodrigues(rvec, R);
	Eigen::Matrix3d e_R;
	e_R << R.at<double>(0), R.at<double>(1), R.at<double>(2),
		R.at<double>(3), R.at<double>(4), R.at<double>(5),
		R.at<double>(6), R.at<double>(7), R.at<double>(8);

	Eigen::Quaternion<double> quaternion(e_R);

	cv::Vec4d q;
	q[0] = quaternion.x();
	q[1] = quaternion.y();
	q[2] = quaternion.z();
	q[3] = quaternion.w();

	return q;
}