#pragma once

#include "IPanel.hpp"

namespace irtgui{

    class ROSPseudoImagePublisherPanel: public IPanel
    {
        public:
            ROSPseudoImagePublisherPanel(std::string name, float width, float height, std::string panelTag, bool open = true) : IPanel(name, width, height, panelTag, open) {}
            ~ROSPseudoImagePublisherPanel() {};
            using Ptr = std::shared_ptr<ROSPseudoImagePublisherPanel>;

            void registerToolUI() override;
            // void setupImagePool(std::shared_ptr<std::map<std::string, sensor_msgs::ImageConstPtr>> image_pool)
            // {
            //     this->image_pool_ = image_pool;
            // }

        private:
            // std::shared_ptr<std::map<std::string, sensor_msgs::ImageConstPtr>> image_pool_ = std::make_shared<std::map<std::string, sensor_msgs::ImageConstPtr>>();
            // std::vector<std::string> subscribed_topics_;
            // std::vector<std::string> available_topics_;
    };
    
}