#include <gui/views/ROSTopicManagerPanel.hpp>

namespace irtgui
{
    void ROSTopicManagerPanel::registerToolUI()
    {
        ImGui::SetNextWindowSize(ImVec2(getWidth(), getHeight()), ImGuiCond_FirstUseEver);
        ImGui::Begin(getName().c_str(), openFlag(), ImGuiWindowFlags_AlwaysAutoResize);
        toolUI();
        ImGui::End();
        ros::spinOnce();
    }

    void ROSTopicManagerPanel::toolUI()
    {
        // this.topicGrabber_.updateTopicList();
        this->topicGrabber_->updateTopicTreeNode();
        ImGui::BeginTabBar("ROSTopicManager");
        if (ImGui::BeginTabItem("Subscribe Topic"))
        {
            // ImGui::Text("Add Topic");
            subscribedTopicsUI();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Publish Topic"))
        {
            ImGui::Text("Publish Topic");
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Service"))
        {
            ImGui::Text("Service");
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Action"))
        {
            ImGui::Text("Action");
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();

    }

    void ROSTopicManagerPanel::subscribedTopicsUI()
    {
        ImGui::Text("Available Topics:");
        this->topicDisplayRootTree_ = this->topicGrabber_->getTopicDisplayRoot();
        if (this->topicDisplayRootTree_ != nullptr)
        {
            createTopicTreeNode(this->topicDisplayRootTree_);

            ImGui::Separator();
            ImGui::Text("Subscribed Topic:");

            if (selectedTopicNode_ != nullptr)
            {
                std::string topicName = selectedTopicNode_->parentField + selectedTopicNode_->value;
                ImGui::Text("Selected Topic: %s", topicName.c_str());

                std::vector<std::string> displayOptions = irtmsg::getDiplayOptionsForTopic(selectedTopicNode_->dataType);
                // ImGui::Text("Display Options:");
                
                irtgui::simpleDropDownMenu("Display Options", displayOptions, *selectedDisplayOption_, [&](std::string selectedOption) {
                    // Handle the selected option
                    // std::cout << "Selected option: " << selectedOption << std::endl;
                    *selectedDisplayOption_ = selectedOption;
                }, 400);



                if (ImGui::Button("Subscribe"))
                {
                    //check data type
                    if (selectedTopicNode_->dataType == "sensor_msgs/Image")
                    {
                        this->topicManagerController_->subscribeNewImageTopic(topicName);

                        this->subscribed_topics_.push_back(topicName);
                    }

                    //check the display option
                    if (*selectedDisplayOption_ == "Image")
                    {
                        //add new panel to the display pool on the main menu
                    }
                }
            }
            else
            {
                ImGui::Text("No topic selected");
            }
        }
        else
        {
            ImGui::Text("No topics available");
        }
        
    }

    void ROSTopicManagerPanel::createTopicTreeNode(std::shared_ptr<irtutils::topicDisplayNode> node)
    {
        if (node->isParent){
            // ImGui::Text(node->value.c_str());
            if (ImGui::TreeNode(node->value.c_str()))
            {
                for (auto child : node->children)
                {
                    createTopicTreeNode(child.second);
                }
                ImGui::TreePop();

            }

        }
        else
        {
            ImGui::Bullet();
            bool isSelected = (selectedTopicNode_ == node);
            if (ImGui::Selectable(node->value.c_str(), isSelected, ImGuiSelectableFlags_SpanAllColumns)) {
                selectedTopicNode_ = node;
            }
            
            // Check if the item is being hovered
            if (ImGui::IsItemHovered())
            {
                ImGui::BeginTooltip();
                // ImGui::Text("Topic Name: %s", node->value.c_str());
                // ImGui::Text("Parent Field: %s", node->parentField.c_str());
                std::string topicName = node->parentField + node->value;
                ImGui::Text("Topic Name: %s", topicName.c_str());
                ImGui::Text("Data Type: %s", node->dataType.c_str());

                ImGui::EndTooltip();
            }
        }
    }

};