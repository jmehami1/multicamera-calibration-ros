#include "ros_camera.h"

RosCamera::RosCamera(ros::NodeHandle nh, std::string topicName, unsigned int cameraIndex, BoardConfiguration board, cv::Mat intrinsic):
nh_(nh), topicName_(topicName), cameraIndex_(cameraIndex), board_(board), intrinsic_(intrinsic), it_(nh)
{
	// Initialise the ArUco board configuration
  	parameters_ = cv::aruco::DetectorParameters::create();
	arucoBoard_ = cv::aruco::Board::create(board.objPoints, board.dictionary, board.ids);

	runThread_ = true;
	thr = std::thread(&RosCamera::imageQueueProcessing, this);

	imageSub_ = it_.subscribe(topicName_, 1000, &RosCamera::cameraImageCallback, this);

	ROS_INFO_STREAM("subscribed to " << topicName_);

}

RosCamera::~RosCamera()
{
	runThread_ = false;
	cv.notify_all();
	thr.join();
}

void RosCamera::cameraImageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  	cv::Mat inputImage = cv_bridge::toCvShare(msg, "bgr8")->image; // Convert sensor_msgs/Image to cv::Mat

  	// cv::Vec4d quaternion; cv::Vec3d tvec;
	auto timeStamp = msg->header.stamp;

	{
		std::unique_lock<std::mutex> lock(mut);
		imageQueue.push(inputImage);
		timeStampQueue.push(timeStamp);
		lock.unlock();
		cv.notify_one();
	}


//   if (arucoBoardDetection(inputImage, quaternion, tvec)){
// 	auto timeStamp = msg->header.stamp;
//   	BoardCameraExtrinsic curExt;
//   	curExt.camIndex = cameraIndex_;
// 	curExt.quaternion = quaternion;
// 	curExt.tvec = tvec;
// 	curExt.stamp = timeStamp;
// 	boardExtrinsics.push_back(curExt);
//   }

  // std::cout << camera_index_ << "," << timeStamp << "," << tvec[0] << "," << tvec[1] << "," << tvec[2] << "," << quaternion[0] << "," << quaternion[1] << "," << quaternion[2] << "," << quaternion[3] << std::endl;
}

void RosCamera::imageQueueProcessing()
{
	while (runThread_){
		
		std::unique_lock<std::mutex> lock(mut);
        cv.wait(lock, [this]{ return !imageQueue.empty(); });

		if (imageQueue.empty())
			continue;

		cv::Mat curImage = imageQueue.front();
		imageQueue.pop();

		ros::Time timeStamp = timeStampQueue.front();
		timeStampQueue.pop();
		lock.unlock();
		

		cv::Vec4d quaternion;
		cv::Vec3d tvec;
		
		if (arucoBoardDetection(curImage, quaternion, tvec)){
			// auto timeStamp = msg->header.stamp;
			BoardCameraExtrinsic curExt;
			curExt.camIndex = cameraIndex_;
			curExt.quaternion = quaternion;
			curExt.tvec = tvec;
			curExt.stamp = timeStamp;
			boardExtrinsics.push_back(curExt);
  		}

		//std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
}

bool RosCamera::arucoBoardDetection(cv::Mat &image, cv::Vec4d &quaternion, cv::Vec3d &tvec)
{
	quaternion = 0;
    tvec = 0;

  	if (!image.empty()){
    	
    	// Marker detection
    	std::vector<int> markerIds; // Create a vector contains marker id
    	std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    	cv::aruco::detectMarkers(image, board_.dictionary, markerCorners, markerIds, parameters_, rejectedCandidates);

		//check if any markers are found
		if (markerIds.size() < 1)
			return false;

    	std::vector<cv::Point3f> objPoints;
    	std::vector<cv::Point2f> imgPoints;
    	cv::aruco::getBoardObjectAndImagePoints(arucoBoard_, markerCorners, markerIds, objPoints, imgPoints);

		//check if atleast four corners of one marker are determined
		if (objPoints.size() < 4)
			return false;
		

    	cv::Vec3d rvec;

		cv::solvePnP(objPoints, imgPoints, intrinsic_, distCoeffs_, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);

    	// if (!objPoints.empty() && !imgPoints.empty())
      	// 	cv::solvePnP(objPoints, imgPoints, intrinsic_, distCoeffs_, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
    	// else 
      	// 	return false;

		quaternion = rvecToQuaternion(rvec);
		// ROS_INFO_STREAM(cameraIndex_ << " POSE FOUND");
		return true;
  	}

 	return false;
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