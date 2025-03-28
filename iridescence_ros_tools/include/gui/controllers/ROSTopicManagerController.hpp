#pragma once

#include <ros/ros.h>
#include <map>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

namespace irtgui{

    class ROSTopicManagerController
    {
    public:
        ROSTopicManagerController(ros::NodeHandle &nh, std::shared_ptr<std::map<std::string, sensor_msgs::ImageConstPtr>> subs_image_pool) : nh_(nh), subs_image_pool_(subs_image_pool) {}
        ~ROSTopicManagerController() {}

    
    private:
        ros::NodeHandle &nh_;
        std::map<std::string, std::shared_ptr<ros::Subscriber>> subscribers_;
        std::map<std::string, std::shared_ptr<ros::Publisher>> publishers_;
        std::shared_ptr<std::map<std::string, sensor_msgs::ImageConstPtr>> subs_image_pool_ = std::make_shared<std::map<std::string, sensor_msgs::ImageConstPtr>>();

    
    public:

        void subscribeNewImageTopic(const std::string &topicName);
        
        void setupSubsImagePool(std::shared_ptr<std::map<std::string, sensor_msgs::ImageConstPtr>> subs_image_pool)
        {
            this->subs_image_pool_ = subs_image_pool;
        }

    


    private:
        void displaySubscribedTopic()
        {
            for (const auto &sub : subscribers_)
            {
                std::cout << "Subscribing: " << sub.first << std::endl;
            }
        }

        void displayPublishedTopic()
        {
            for (const auto &pub : publishers_)
            {
                std::cout << "Publishing: " << pub.first << std::endl;
            }
        }

        bool checkIfSubscribedTopicExists(const std::string &topicName)
        {
            return subscribers_.find(topicName) != subscribers_.end();
        }
        bool checkIfPublishedTopicExists(const std::string &topicName)
        {
            return publishers_.find(topicName) != publishers_.end();
        }   

    };




}