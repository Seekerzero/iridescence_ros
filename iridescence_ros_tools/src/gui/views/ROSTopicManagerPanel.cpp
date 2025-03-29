#include <gui/views/ROSTopicManagerPanel.hpp>

namespace irtgui
{
    void ROSTopicManagerPanel::registerToolUI()
    {
        //here we re-publish pseudo images
        for (auto &published_topic : this->published_topics_)
        {
            if (published_topic.second.second)
            {
                //check if the topic is already published
                if (this->pub_image_pool_->find(published_topic.first) != this->pub_image_pool_->end())
                {
                    //update the image
                    this->topicManagerController_->updatePseduoImagePublish(published_topic.first);
                }
            }
        }

        ImGui::SetNextWindowSize(ImVec2(getWidth(), getHeight()), ImGuiCond_FirstUseEver);
        ImGui::Begin(getName().c_str(), openFlag(), ImGuiWindowFlags_AlwaysAutoResize);
        toolUI();
        ImGui::End();
        // ros::spinOnce();
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
            // ImGui::Text("Publish Topic");
            publishedTopicsUI();
            ImGui::Separator();
            pseudoImagePublisherUI();
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

    void ROSTopicManagerPanel::publishedTopicsUI()
    {
        ImGui::Text("Published Topics:");
        // ImGui::Text("No topics available");
    }

    void ROSTopicManagerPanel::pseudoImagePublisherUI()
    {
        ImGui::Text("Select a image from file");
        if (this->pseudo_img_pub_selected_image_file_name_.empty())
        {
            ImGui::Text("No image selected");
        }
        else
        {
            ImGui::Text("Selected image: %s", pseudo_img_pub_selected_image_file_name_.c_str());
        }

        if (ImGui::Button("Select Image"))
        {
            std::vector<std::string> file_path = pfd::open_file("Select Image", "Image Files (*.png *.jpg *.jpeg *.bmp);;All Files (*)").result();
            if (!file_path.empty())
            {
                pseudo_img_pub_selected_image_file_name_ = file_path[0].substr(file_path[0].find_last_of("/\\") + 1);
                pseudo_img_pub_selected_image_file_path_ = file_path[0];
            }
        }

        ImGui::InputText("Publish Topic Name", &pseudo_img_pub_publish_topic_name_[0], 256);
        
        pseudo_img_pub_publish_topic_name_ = pseudo_img_pub_publish_topic_name_.c_str();
        //check if image is selected, check if topic name is not empty otherwise disable button
        bool button_enabled = !pseudo_img_pub_selected_image_file_name_.empty() && (strcmp(pseudo_img_pub_publish_topic_name_.c_str(), "") != 0);

        if (pseudo_img_pub_selected_image_file_name_.empty())
        {
            ImGui::Text("No image selected");
        }
        if (strcmp(pseudo_img_pub_publish_topic_name_.c_str(), "") == 0)
        {
            ImGui::Text("No topic name selected");
        }

        if (button_enabled){
            if (ImGui::Button("Publish Image"))
            {
                // std::cout << "Publishing image: " << pseudo_img_pub_publish_topic_name_ << std::endl;
                // Load the image
                // cv::Mat image = cv::imread(pseudo_img_pub_selected_image_file_path_, cv::IMREAD_UNCHANGED);
                //load the image using cv_bridge
                // cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage());
                cv::Mat image = cv::imread(pseudo_img_pub_selected_image_file_path_, cv::IMREAD_COLOR);
                // std::cout << "Channels: " << image.channels() << std::endl;
                // std::cout << "Depth: " << image.depth() << std::endl;
                std::string encoding =  "bgr8";

                try
                {
                    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), encoding, image).toImageMsg();

                    this->pub_image_pool_->insert(std::make_pair(pseudo_img_pub_publish_topic_name_, msg));
                    this->published_topics_.insert(std::make_pair(pseudo_img_pub_publish_topic_name_, std::make_pair("sensor_msgs/Image", true)));
                    this->topicManagerController_->publishPseudoImageTopic(pseudo_img_pub_publish_topic_name_);                    

                }
                catch (const std::exception& e)
                {
                    // ROS_ERROR("Failed to convert image: %s", e.what());
                    std::cout << "Failed to convert image: " << e.what() << std::endl;
                    return;
                }


            }
        }
        else
        {
            ImGui::BeginDisabled();
            ImGui::Button("Publish Image");
            ImGui::EndDisabled();
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