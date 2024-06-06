#ifndef TOOLVIEWER_HPP
#define TOOLVIEWER_HPP

#include <ros/ros.h>
#include <memory>
#include <utils/TopicGrabber.hpp>
#include <glk/primitives/primitives.hpp>
#include <guik/viewer/light_viewer.hpp>

namespace irtools{

    class ToolViewer{
        private:
            guik::LightViewer* viewer_;
            ros::NodeHandle nh_;
            TopicGrabber topicGrabber_{nh_};
            std::vector<std::string> subscribed_topics_;
            std::vector<std::string> available_topics_;

            bool show_add_topic_window = false;
            std::string drop_list_selected_topic = "Select a topic";
        public:
            ToolViewer(){
                viewer_ = guik::LightViewer::instance();
                register_ui_callback();
            };

            virtual ~ToolViewer(){};
            using Ptr = std::shared_ptr<ToolViewer>;


            void register_ui_callback(){
                viewer_->register_ui_callback("ui", [&]() {
                ImGui::Begin("IRS Tools Menu",0,ImGuiWindowFlags_AlwaysAutoResize);
                defaultUI();
                ImGui::End();
                });
            }

            void updateTopicList(){
                available_topics_ = topicGrabber_.getTopics();
            }

            void defaultUI();

            void updateSubscribeDrawables();

            void addTopicWindow();

            void spin(){
                while (viewer_->spin_once()) {
                    updateSubscribeDrawables();
                    ros::spinOnce();
                }
            }
    };
}




#endif // TOOLVIEWER_HPP