#ifndef GUI_SYNTAX_DELETIONPANEL_HPP
#define GUI_SYNTAX_DELETIONPANEL_HPP

#include <guik/viewer/light_viewer.hpp>
#include <iostream>
#include <map>
#include <vector>
#include <Eigen/Core>
#include <gui/controllers/GeometriesDemoController.hpp>


namespace gui{

    static bool pointDeletionPanel(std::string label, std::shared_ptr<PointMessage> pointBuffer){

        ImGui::BeginGroup();
        if (pointBuffer->pointId != -1){
            if (ImGui::Button(("Delete##" + label).c_str())){
                pointBuffer->action = actionType::DELETE;
                GeometriesDemoController::getInstance().updatePointMessage(pointBuffer);
                return true;
            }       
        }
        else{
            //if the point id is -1, we darken the button
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.5f, 0.5f, 0.5f, 1.0f));
            ImGui::Button(("Delete##" + label).c_str());
            ImGui::PopStyleColor();
        }

        ImGui::EndGroup();

        return false;
    }

}













#endif // GUI_SYNTAX_DELETIONPANEL_HPP