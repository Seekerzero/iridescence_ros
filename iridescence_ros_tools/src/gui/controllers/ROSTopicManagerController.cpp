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

}