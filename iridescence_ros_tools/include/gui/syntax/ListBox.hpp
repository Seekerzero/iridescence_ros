#ifndef GUI_SYNTAX_LISTBOX_HPP
#define GUI_SYNTAX_LISTBOX_HPP

#include <guik/viewer/light_viewer.hpp>
#include <iostream>
#include <map>
#include <vector>



namespace gui{

    static void selectableListBox(std::string label, std::vector<std::string> itemNameList, int& selectedIndex){
        ImGui::Text("%s", label.c_str());
        if (ImGui::BeginListBox(("##" + label).c_str())){
            for (int i = 0; i < itemNameList.size(); i++){
                bool isSelected = selectedIndex == i;
                if (ImGui::Selectable(itemNameList[i].c_str(), isSelected)){
                    selectedIndex = i;
                }
                if (isSelected){
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndListBox();
        }

    }

    static void selectableListBoxWithSelectableName(std::string label, std::vector<std::string> itemNameList, std::string& selectedName, int& selectedIndex){
        ImGui::Text("%s", label.c_str());
        if (ImGui::BeginListBox(("##" + label).c_str())){
            for (int i = 0; i < itemNameList.size(); i++){
                bool isSelected = selectedIndex == i;
                if (ImGui::Selectable(itemNameList[i].c_str(), isSelected)){
                    selectedIndex = i;
                    selectedName = itemNameList[i];
                }
                if (isSelected){
                    ImGui::SetItemDefaultFocus();
                    selectedName = itemNameList[i];
                }
            }
            ImGui::EndListBox();
        }

    }

}












#endif // GUI_SYNTAX_LISTBOX_HPP