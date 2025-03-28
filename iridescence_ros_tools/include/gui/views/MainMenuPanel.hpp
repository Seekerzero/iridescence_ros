#pragma once

#include "IPanel.hpp"
#include <config/MainMenuPanelConfig.hpp>
#include "ROSTopicManagerPanel.hpp"
#include <gui/syntax/ROSImageDisplay.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <map>

namespace irtgui
{

    class MainMenuPanel : public IPanel
    {
    public:
        MainMenuPanel(std::string name, float width, float height, std::string panelTag, bool open = true) : IPanel(name, width, height, panelTag, open) {
            this->topic_manager_panel_->setupSubsImagePool(this->subs_image_pool_);
        }
        ~MainMenuPanel() {};
        using Ptr = std::shared_ptr<MainMenuPanel>;

        void registerToolUI() override;

    private:
        irtconfig::MainMenuPanelConfig config_;
        ROSTopicManagerPanel::Ptr topic_manager_panel_ = std::make_shared<ROSTopicManagerPanel>("ROS Topic Manager", 300, 400, "ROSTopicManagerPanel", false);
        // bool show_ros_topic_manager_ = false;
        std::shared_ptr<std::map<std::string, sensor_msgs::ImageConstPtr>> subs_image_pool_ = std::make_shared<std::map<std::string, sensor_msgs::ImageConstPtr>>();
    private:
        void toolUI() override;
        void robotControlModeUI();
        void menuBarUI();
    };

}
