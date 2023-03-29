#include "ros/ros.h"
#include <vector>
#include <iostream>
#include <fstream>
#include "yaml_reader.h"
#include <filesystem>
#include <thread>
#include <ros/master.h>
#include "ros_camera.h"
#include <chrono>



namespace fs = std::filesystem;
using namespace std::chrono_literals;

// Function to write vector to csv file 
void writeVectorToCSV(std::string cameraInfoFile, std::vector<BoardCameraExtrinsic> &vec, std::vector<std::string> topicNames) 
{ 
   std::cout << "\nWriting to csv ... " << std::flush;
    std::ofstream outfile; 
    outfile.open(cameraInfoFile); 

    for (int i = 0; i < vec.size(); i++) { 

        // If the element is the last in the vector, don't add a comma after it.  
        if (i == vec.size() - 1) {  

            outfile << vec.at(i).camIndex << "," << vec.at(i).stamp;
            
            auto vecT = vec.at(i).tvec;
            for (int i = 0; i < 3; i++)
                outfile << "," << vecT(i);

            auto vecQ = vec.at(i).quaternion;
            for (int i = 0; i < 4; i++)
                outfile << "," << vecQ(i);

            outfile << "," << topicNames.at(vec.at(i).camIndex);

            outfile << "\n";  

        } else {
            outfile << vec.at(i).camIndex << "," << vec.at(i).stamp;
            
            auto vecT = vec.at(i).tvec;
            for (int i = 0; i < 3; i++)
                outfile << "," << vecT(i);

            auto vecQ = vec.at(i).quaternion;
            for (int i = 0; i < 4; i++)
                outfile << "," << vecQ(i);

            outfile << "," << topicNames.at(vec.at(i).camIndex);

            outfile << "," << "\n";  
        }  

    }  

    outfile.close(); 
}

//thread process function to automatically shutdown node is image topics are no longer active
void checkTimeoutCameraTopicsProcess(std::vector<std::shared_ptr<RosCamera>> cameras, const int timeOut, const int numTopics) {

  int numTopicsTimedOut = 0;

  while (ros::ok){

      auto curTime = std::chrono::high_resolution_clock::now();

      for (auto c : cameras){
        auto lastMsgTime = c->getLasMsgTime();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(curTime - lastMsgTime);

        if (duration.count() > timeOut)
            numTopicsTimedOut++;
      }
      
      ROS_INFO_STREAM("Number of active topic streams: " << numTopics - numTopicsTimedOut << " out of " << numTopics);
        
      if (numTopicsTimedOut >= numTopics){
        std::cout << "\nShutting down\n" << std::endl;
        ros::shutdown();
        break;
      }else
        numTopicsTimedOut = 0;
    
    std::this_thread::sleep_for(2000ms);
  }

  ros::shutdown();

}



int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "multi_camera_calibration");

  if (!ros::master::check()){
    std::cerr << "roscore is not running" << std::endl;
    exit(1);
  }

  //calibration file directory
  fs::path calibrationFiles(__FILE__);
  calibrationFiles = calibrationFiles.parent_path();
  calibrationFiles = calibrationFiles.parent_path();
  calibrationFiles = calibrationFiles / "calibration_files";

  //check for aruco board marker YAML file 
  fs::path aruco_board_markers = calibrationFiles / "aruco-board-markers.yaml";
  if (!fs::exists(aruco_board_markers)){
    std::cerr << "ERROR Does not exist: " << aruco_board_markers.string() << std::endl;
    exit(1);
  }

  //check for camera info YAML file 
  fs::path cameraInfoFile = calibrationFiles / "camera_info.yaml";
  if (!fs::exists(cameraInfoFile)){
    std::cerr << "ERROR Does not exist: " << cameraInfoFile.string() << std::endl;
    exit(1);
  }

  ros::NodeHandle nh;

  const auto processor_count = std::thread::hardware_concurrency();
  std::cout << "Number of processors available: " << processor_count << std::endl;

  //Read camera intrincs and topic names from camera info YAML
  std::cout << "Reading camera info file..." << std::endl;
  std::vector<cv::Mat> intrinsic_vectors = yaml::Read_Intrinsic(cameraInfoFile);
  auto cameraTopicNames = yaml::Read_Topic_Names(cameraInfoFile);

  std::cout << "Reading ArUco board file..." << std::endl;
  BoardConfiguration aruco_board;
  yaml::Read_ArUco(aruco_board_markers.string(), aruco_board.dictionary, aruco_board.ids, aruco_board.objPoints);

  std::cout << "\n" << std::endl;

  //create ros camera objected using each image topic
  std::vector<std::shared_ptr<RosCamera>> cameras;
  for (int i = 0; i < cameraTopicNames.size(); i++){
    cameras.push_back(std::shared_ptr<RosCamera>(new RosCamera(nh, cameraTopicNames.at(i), i, aruco_board, intrinsic_vectors.at(i))));
  }

  int timeOut = 10;

  std::cout << "\nNode will timeout and quit after " << timeOut << " seconds of inactivity from topics!\n" << std::endl; 
  std::thread timeOutT(checkTimeoutCameraTopicsProcess, cameras, timeOut, cameraTopicNames.size()); 

  //use all available processors to handle image callbacks
  ros::AsyncSpinner spinner(processor_count-1);
  spinner.start();
  ros::waitForShutdown();

  timeOutT.join();

  //collect all successfully estimated board poses from each camera
  std::vector<BoardCameraExtrinsic> allData;

  int i = 0;
  for (auto c : cameras){
    std::vector<BoardCameraExtrinsic> curData = c->getCopyBoardExtrinsicData();
    std::cout << "Camera " << i << " has " << curData.size() << " poses" << std::endl;
    i++;
    allData.insert(std::end(allData), std::begin(curData), std::end(curData));
  }

  //save all poses to CSV file
  std::cout << "Number of estimated poses across all cameras: " << allData.size() << std::endl;
  fs::path cameraPosesCSV = calibrationFiles / "camera_board_poses_timestamped.csv";
  writeVectorToCSV(cameraPosesCSV.string(), allData, cameraTopicNames);

  std::cout << "Done!!!"  << std::endl;

  return 0;
}
