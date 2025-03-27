#pragma once

#include "IPanel.hpp"
#include <ros/ros.h>
#include <utils/ros/TopicGrabber.hpp>
#include <messages/gui/ROSTopicDisplayMapping.hpp>
#include <gui/syntax/DropDownMenu.hpp>
#include <gui/controllers/ROSTopicManagerController.hpp>

namespace irtgui
{
    class ROSTopicManagerPanel : public IPanel
    {
        public:
            ROSTopicManagerPanel(std::string name, float width, float height, std::string panelTag, bool open = true) : IPanel(name, width, height, panelTag, open) {}
            ~ROSTopicManagerPanel() {};
            using Ptr = std::shared_ptr<ROSTopicManagerPanel>;

            void registerToolUI() override;
            void setupImagePool(std::shared_ptr<std::map<std::string, sensor_msgs::ImageConstPtr>> image_pool)
            {
                this->image_pool_ = image_pool;
                this->topicManagerController_->setupImagePool(this->image_pool_);
            }

        private:
            ros::NodeHandle nh_;
            std::shared_ptr<std::map<std::string, sensor_msgs::ImageConstPtr>> image_pool_ = std::make_shared<std::map<std::string, sensor_msgs::ImageConstPtr>>();
            std::shared_ptr<irtutils::TopicGrabber> topicGrabber_ = std::make_shared<irtutils::TopicGrabber>(nh_);
            std::shared_ptr<ROSTopicManagerController> topicManagerController_ = std::make_shared<ROSTopicManagerController>(nh_, image_pool_);

            
            std::vector<std::string> subscribed_topics_;
            std::vector<std::string> available_topics_;
            std::shared_ptr<irtutils::topicDisplayNode> topicDisplayRootTree_ = std::make_shared<irtutils::topicDisplayNode>("init", "", true);
            std::shared_ptr<irtutils::topicDisplayNode> selectedTopicNode_ = nullptr;
            std::shared_ptr<std::string> selectedDisplayOption_ = std::make_shared<std::string>("");

        private:
            void toolUI() override;
            void subscribedTopicsUI();
            void createTopicTreeNode(std::shared_ptr<irtutils::topicDisplayNode> node);
    };

}