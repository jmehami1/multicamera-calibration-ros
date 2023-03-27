// #include "ros/ros.h"
// #include "camera_wrapper.h"
// #include "camera.h"
// #include "calibration.h"
// #include "wrapper.h"
#include <vector>
#include <iostream>
#include <fstream>
#include "yaml_reader.h"
#include <filesystem>

#include "ros_camera.h"

namespace fs = std::filesystem;


// Function to write vector to csv file 
void writeVectorToCSV(std::string cameraInfoFile, std::vector<BoardCameraExtrinsic> &vec, std::vector<std::string> topicNames) 
{ 
   std::cout << "writing csv ..." <<  std::endl;
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



int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera_calibration");

  // ros process input argument
  ros::NodeHandle nh;

  //calibration files
  fs::path calibrationFiles(__FILE__);
  calibrationFiles = calibrationFiles.parent_path();
  calibrationFiles = calibrationFiles.parent_path();
  calibrationFiles = calibrationFiles / "calibration_files";

  //check for aruco board marker YAML file 
  fs::path aruco_board_markers = calibrationFiles / "aruco-board-markers.yaml";
  if (!fs::exists(aruco_board_markers))
    std::cerr << "ERROR Does not exist: " << aruco_board_markers.string() << std::endl;

  //check for camera info YAML file 
  fs::path cameraInfoFile = calibrationFiles / "camera_info.yaml";
  if (!fs::exists(cameraInfoFile))
    std::cerr << "ERROR Does not exist: " << cameraInfoFile.string() << std::endl;

  //Read camera intrincs and topic names from camera info YAML
  std::cout << "Reading camera info file..." << std::endl;
  std::vector<cv::Mat> intrinsic_vectors = yaml::Read_Intrinsic(cameraInfoFile);
  auto cameraTopicNames = yaml::Read_Topic_Names(cameraInfoFile);

  std::cout << "Reading ArUco board file..." << std::endl;
  BoardConfiguration aruco_board;
  yaml::Read_ArUco(aruco_board_markers.string(), aruco_board.dictionary, aruco_board.ids, aruco_board.objPoints);

  std::vector<std::shared_ptr<RosCamera>> cameras;

  for (int i = 0; i < cameraTopicNames.size(); i++){

    // cameras->push_back(new RosCamera(nh, cameraTopicNames.at(i), i, aruco_board, intrinsic_vectors.at(i)));

    cameras.push_back(std::shared_ptr<RosCamera>(new RosCamera(nh, cameraTopicNames.at(i), i, aruco_board, intrinsic_vectors.at(i))));
    // std::shared_ptr<CameraInterface> cam(new Camera(i));
    // cams.push_back(cam);

    // std::shared_ptr<CameraWrapper> cam_wrp(new CameraWrapper(nh, cameraTopicNames.at(i), i));
    // cam_wrp->setCamera(cam);
    // camWrappers.push_back(cam_wrp);
  }

//   // std::shared_ptr<CameraInterface> cam(new Camera(index));
//   // std::shared_ptr<CameraWrapper> cam_wrp(new CameraWrapper(nh, index));
//   // cam_wrp->setCamera(cam);

  ros::spin();

  std::cout << "Collecting all poses and saving to CSV file" << std::endl;

  std::vector<BoardCameraExtrinsic> allData;

  int i = 0;
  for (auto c : cameras){
    std::vector<BoardCameraExtrinsic> curData = c->getCopyBoardExtrinsicData();
    std::cout << "Camera " << i << " has " << curData.size() << " poses" << std::endl;
    i++;
    allData.insert(std::end(allData), std::begin(curData), std::end(curData));
  }

  std::cout << "Number of estimated poses across all cameras: " << allData.size() << std::endl;

  fs::path cameraPosesCSV = calibrationFiles / "camera_board_poses_timestamped.csv";

  writeVectorToCSV(cameraPosesCSV.string(), allData, cameraTopicNames);
//     // allData.push_back();
//     // a.insert(std::end(a), std::begin(b), std::end(b));

// // function to write vector to csv file

  std::cout << "Shutting down!!!" << allData.size() << std::endl;


  ros::shutdown();

  return 0;
}

// function that loads a rosbag

// function that determines the closest ros message using time between two ros topics