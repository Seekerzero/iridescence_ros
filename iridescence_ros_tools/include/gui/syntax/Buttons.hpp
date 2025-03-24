#ifndef GUI_SYNTAX_BUTTONS_HPP
#define GUI_SYNTAX_BUTTONS_HPP

#include <guik/viewer/light_viewer.hpp>

namespace gui
{
    static void createButton(const std::string &label, const char *tooltipString, const std::function<void()> &callback)
    {

        if (ImGui::Button(label.c_str()))
        {
            callback();
        }
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("%s", tooltipString);
        }
    }

    static void createIconButtonSwitch(bool &buttonStatus, bool &otherButtonStatus, const char *icon1, const char *icon2, const char *tooltipString1, const char *tooltipString2, const std::function<void()> &onClick, ImVec2 size = ImVec2(40, 40), ImVec4 color1 = ImVec4(0.0f, 0.5f, 0.0f, 1.0f), ImVec4 color2 = ImVec4(0.5f, 0.0f, 0.0f, 1.0f))
    {
        if (buttonStatus)
        {
            ImGui::PushStyleColor(ImGuiCol_Button, color1);
            if (ImGui::Button((icon1 + std::string("##") + "icon").c_str(), size))
            {
                buttonStatus = false;
                otherButtonStatus = !buttonStatus;
                onClick();
            }
            ImGui::PopStyleColor();
            if (ImGui::IsItemHovered())
            {
                ImGui::SetTooltip("%s", tooltipString1);
            }
        }
        else
        {
            ImGui::PushStyleColor(ImGuiCol_Button, color2);
            if (ImGui::Button((icon2 + std::string("##") + "icon").c_str(), size))
            {
                buttonStatus = true;
                otherButtonStatus = !buttonStatus;
                onClick();
            }
            ImGui::PopStyleColor();
            if (ImGui::IsItemHovered())
            {
                ImGui::SetTooltip("%s", tooltipString2);
            }
        }
    }

    static void createIconButton(const char *icon, const char *tooltipString, const std::function<void()> &onClick, ImVec2 size = ImVec2(40, 40))
    {
        if (ImGui::Button((icon + std::string("##") + "icon").c_str(), size))
        {
            onClick();
        }
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("%s", tooltipString);
        }
    }

}

#endif // GUI_SYNTAX_BUTTONS_HPP