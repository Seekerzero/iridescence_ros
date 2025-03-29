#include <gui/views/MainMenuPanel.hpp>

namespace irtgui
{
    void MainMenuPanel::registerToolUI()
    {   
        if (this->topic_manager_panel_->isOpen())
        {
            this->topic_manager_panel_->registerToolUI();
        }

        for (auto image : *this->subs_image_pool_)
        {
            if (image.second != nullptr)
            {
              updateROSImageDisplay(image.second, image.first, guik::LightViewer::instance());   
            }
        }

        ImGui::SetNextWindowSize(ImVec2(getWidth(), getHeight()), ImGuiCond_FirstUseEver);
        ImGui::Begin(getName().c_str(), NULL, ImGuiWindowFlags_MenuBar);
        toolUI();
        ImGui::End();
        ros::spinOnce();
    }

    void MainMenuPanel::toolUI()
    {   
        menuBarUI();
        //TODO: add load configuration when there is a configuration file
        //TODO: add save configuration
        robotControlModeUI();
        //TODO: add ImageDisplay Control UI when there is a image display
    }

    void MainMenuPanel::robotControlModeUI()
    {
        ImGui::Text("Robot Control Mode");

        ImGui::RadioButton("Manual", (int *)&config_.robot_control_mode, (int)irtconfig::RobotControlMode::MANUAL);

        ImGui::SameLine();

        ImGui::RadioButton("Auto", (int *)&config_.robot_control_mode, (int)irtconfig::RobotControlMode::AUTO);
    }

    void MainMenuPanel::menuBarUI()
    {
        if (ImGui::BeginMenuBar())
        {
            auto createMenu = [&](const char *menuName, const std::vector<std::pair<std::string, std::function<void()>>> &items)
            {
                if (ImGui::BeginMenu(menuName))
                {
                    for (const auto &item : items)
                    {
                        if (ImGui::MenuItem(item.first.c_str()))
                        {
                            item.second();
                        }
                    }
                    ImGui::EndMenu();
                }
            };

            createMenu("ROS", {{"Topic Manager", [&]()
                                 { this->topic_manager_panel_->setOpenFlag(true);}},
                                {"Pseudo Image Publisher", [&]()
                                 {/*something publish image*/}}});

            ImGui::EndMenuBar();
        }
    }


}