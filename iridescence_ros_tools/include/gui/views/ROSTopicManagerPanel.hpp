#pragma once

#include "IPanel.hpp"
#include <ros/ros.h>
#include <utils/ros/TopicGrabber.hpp>
#include <messages/gui/ROSTopicDisplayMapping.hpp>
#include <gui/syntax/DropDownMenu.hpp>
#include <gui/controllers/ROSTopicManagerController.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
// #include <opencv4/opencv2/imgcodecs/legacy/constants_c.h>

namespace irtgui
{
    class ROSTopicManagerPanel : public IPanel
    {
        public:
            ROSTopicManagerPanel(std::string name, float width, float height, std::string panelTag, bool open = true) : IPanel(name, width, height, panelTag, open) {}
            ~ROSTopicManagerPanel() {};
            using Ptr = std::shared_ptr<ROSTopicManagerPanel>;

            void registerToolUI() override;
            void setupSubsImagePool(std::shared_ptr<std::map<std::string, sensor_msgs::ImageConstPtr>> subs_image_pool)
            {
                this->subs_image_pool_ = subs_image_pool;
                this->topicManagerController_->setupSubsImagePool(this->subs_image_pool_);
            }

            void setupPubImagePool(std::shared_ptr<std::map<std::string, sensor_msgs::ImageConstPtr>> pub_image_pool)
            {
                this->pub_image_pool_ = pub_image_pool;
                this->topicManagerController_->setupPubImagePool(this->pub_image_pool_);
            }

        private:
            ros::NodeHandle nh_;
            std::shared_ptr<std::map<std::string, sensor_msgs::ImageConstPtr>> subs_image_pool_ = std::make_shared<std::map<std::string, sensor_msgs::ImageConstPtr>>();
            std::shared_ptr<std::map<std::string, sensor_msgs::ImageConstPtr>> pub_image_pool_ = std::make_shared<std::map<std::string, sensor_msgs::ImageConstPtr>>();

            std::shared_ptr<irtutils::TopicGrabber> topicGrabber_ = std::make_shared<irtutils::TopicGrabber>(nh_);
            std::shared_ptr<ROSTopicManagerController> topicManagerController_ = std::make_shared<ROSTopicManagerController>(nh_, subs_image_pool_);

            
            std::vector<std::string> subscribed_topics_;
            // <topic_name, <type, pseudo>>
            std::map<std::string, std::pair<std::string, bool>> published_topics_;
            std::vector<std::string> available_topics_;
            std::shared_ptr<irtutils::topicDisplayNode> topicDisplayRootTree_ = std::make_shared<irtutils::topicDisplayNode>("init", "", true);
            std::shared_ptr<irtutils::topicDisplayNode> selectedTopicNode_ = nullptr;
            std::shared_ptr<std::string> selectedDisplayOption_ = std::make_shared<std::string>("");


            std::string pseudo_img_pub_selected_image_file_name_;
            std::string pseudo_img_pub_selected_image_file_path_;
            std::string pseudo_img_pub_publish_topic_name_;

        private:
            void toolUI() override;
            void subscribedTopicsUI();
            void publishedTopicsUI();
            void pseudoImagePublisherUI();
            void createTopicTreeNode(std::shared_ptr<irtutils::topicDisplayNode> node);
    };

}