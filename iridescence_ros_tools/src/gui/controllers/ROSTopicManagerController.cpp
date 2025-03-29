#include <gui/controllers/ROSTopicManagerController.hpp>

namespace irtgui{

    void ROSTopicManagerController::subscribeNewImageTopic(const std::string &topicName)
    {   

        if (checkIfSubscribedTopicExists(topicName))
        {
            std::cout << "Already subscribed to topic: " << topicName << std::endl;
            return;
        }
        else
        {
            //setup callback function
            auto callback = [this, topicName](const sensor_msgs::ImageConstPtr &msg)
            {
                // std::cout << "Received image on topic: " << topicName << std::endl;
                // std::cout << "Image width: " << msg->width << ", height: " << msg->height << std::endl;
                (*subs_image_pool_)[topicName] = msg;
            };
            auto sub = nh_.subscribe<sensor_msgs::Image>(topicName, 1, callback);
            subscribers_[topicName] = std::make_shared<ros::Subscriber>(sub);
            displaySubscribedTopic();
        }
    }

    void ROSTopicManagerController::publishPseudoImageTopic(const std::string &topicName)
    {
        if (checkIfPublishedTopicExists(topicName))
        {
            std::cout << "Already published to topic: " << topicName << std::endl;
            return;
        }
        else
        {
            auto pub = nh_.advertise<sensor_msgs::Image>(topicName, 1);
            publishers_[topicName] = std::make_shared<ros::Publisher>(pub);

            // std::cout << "Publishing to topic: " << topicName << std::endl;
            displayPublishedTopic();
        }
    }

    void ROSTopicManagerController::updatePseduoImagePublish(const std::string &topicName)
    {
        if (pubs_image_pool_->find(topicName) != pubs_image_pool_->end())
        {
            //check if the publisher exists
            if (publishers_.find(topicName) == publishers_.end())
            {
                std::cout << "Publisher not found for topic: " << topicName << std::endl;
                return;
            }
            auto const_msg = (*pubs_image_pool_)[topicName];
            
            // Create a non-const copy of the message
            auto msg = boost::make_shared<sensor_msgs::Image>(*const_msg);
            
            //generate a new header
            // msg->header = std_msgs::Header();
            //publish the image
            publishers_[topicName]->publish(msg);
        }
        else
        {
            std::cout << "No image found in the pool for topic: " << topicName << std::endl;
        }
    }

}