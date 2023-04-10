#ifndef YAML_READER_H
#define YAML_READER_H

#include <vector>
#include <string>
#include <opencv2/aruco/dictionary.hpp>
#include <yaml-cpp/yaml.h>
#include <unistd.h>
#include <iostream>

/**
 * @brief YAML file namespace
 * 
 */
namespace yaml
{
    /**
     * @brief Reads the intrinsic parameters from a YAML file
     * 
     * @param filename 
     * @return std::vector<cv::Mat> 
     */
    std::vector<cv::Mat> Read_Intrinsic(const std::string filename);

    /**
     * @brief Reads the topic names from a YAML file
     * 
     * @param filename 
     * @return std::vector<std::string> 
     */
    std::vector<std::string> Read_Topic_Names(const std::string filename);

    /**
     * @brief Reads ArUco board parameters stored inside of a YAML file
     * 
     * @param fileName 
     * @param dictionary 
     * @param ids 
     * @param objPoints 
     * @return true 
     * @return false 
     */
    bool Read_ArUco(const std::string &fileName, cv::Ptr<cv::aruco::Dictionary> &dictionary,
                    std::vector<int> &ids, std::vector<std::vector<cv::Point3f>> &objPoints);
}

#endif