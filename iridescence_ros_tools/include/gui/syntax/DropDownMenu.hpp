#pragma once

#include <guik/viewer/light_viewer.hpp>
#include <map>
#include <vector>
#include <memory>

namespace irtgui
{

    static bool simpleDropDownMenu(std::string name, std::vector<std::string> items, std::string &selectedItem, std::function<void(std::string)> callback, float width = 0)
    {
        bool changed = false;
        std::string oldSelectedItem = selectedItem;
        if (width > 0)
        {
            ImGui::SetNextItemWidth(width);
        }
        else
        {
            ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
        }
        if (ImGui::BeginCombo(name.c_str(), selectedItem.c_str()))
        {
            for (const auto &item : items)
            {
                if (ImGui::Selectable(item.c_str(), selectedItem == item))
                {
                    selectedItem = item;
                    callback(item);
                }
            }
            ImGui::EndCombo();
        }

        if (oldSelectedItem != selectedItem)
        {
            changed = true;
        }

        return changed;
    }

    template <typename EnumType, typename StringCastFunc, typename CallbackFunc>
    static bool simpleDropDownMenuENUM(std::string name, const std::vector<EnumType> &items, EnumType &selectedEnum, StringCastFunc stringCast, CallbackFunc callback)
    {
        bool changed = false;
        EnumType oldSelectedEnum = selectedEnum;
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
        if (ImGui::BeginCombo(name.c_str(), stringCast(selectedEnum).c_str()))
        {
            for (const auto &item : items)
            {
                if (ImGui::Selectable(stringCast(item).c_str(), selectedEnum == item))
                {
                    selectedEnum = item;
                    callback(item);
                }
            }
            ImGui::EndCombo();
        }
        if (oldSelectedEnum != selectedEnum)
        {
            changed = true;
        }

        return changed;
    }

}
