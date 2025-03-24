#pragma once

#include <ros/ros.h>
//also include ros message types
#include <string>
#include <vector>


namespace irtmsg{

    inline std::vector<std::string> getDiplayOptionsForTopic(const std::string& topicType){
        std::vector<std::string> displayOptions;
        if (topicType == "sensor_msgs/Image"){
            displayOptions.push_back("Image");
        }
        else if (topicType == "sensor_msgs/PointCloud2"){
            displayOptions.push_back("Point Cloud");
        }
        else if (topicType == "sensor_msgs/LaserScan"){
            displayOptions.push_back("Laser Scan");
        }
        else{
            displayOptions.push_back("Unknown");
        }
        return displayOptions;
    }
}