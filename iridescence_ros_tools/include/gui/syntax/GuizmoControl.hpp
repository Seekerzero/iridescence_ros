#ifndef GUI_SYNTAX_GUIZMOCONTROL_HPP
#define GUI_SYNTAX_GUIZMOCONTROL_HPP

#include <guik/viewer/light_viewer.hpp>
#include <guik/model_control.hpp>   
#include <iostream>
#include <map>
#include <vector>
#include <Eigen/Core>
#include <gui/controllers/GeometriesDemoController.hpp>



namespace gui{

    static std::shared_ptr<guik::ModelControl> initGuizmoControl(std::string PanelName, Eigen::Matrix4f transform){
        return std::make_shared<guik::ModelControl>(PanelName, transform);
    }
    
    static bool renderGuizmo(std::string drawableName, std::shared_ptr<Eigen::Matrix4f> transform, std::shared_ptr<guik::ModelControl> modelControlNode, std::string& operationMode, float guizmoScale = 0.1f){
        ImGui::BeginGroup();
        // ImGui::Text("Control Mode");
        ImGui::SameLine();

        //here we set the gizmo transform matrix
        modelControlNode->set_model_matrix(*transform);
        //then we set the gizmo mode
        modelControlNode->set_gizmo_operation(operationMode);

        //here we set the gizmo scale
        modelControlNode->set_gizmo_clip_scale(guizmoScale);

        modelControlNode->set_gizmo_enabled(true);

        // //here we want to make a dropdown menu to select the operation mode
        // if (ImGui::BeginCombo("##operationMode", operationMode.c_str())){
        //     if (ImGui::Selectable("TRANSLATE", operationMode == "TRANSLATE")){
        //         operationMode = "TRANSLATE";
        //         modelControlNode->set_gizmo_operation("TRANSLATE");
        //     }
        //     if (ImGui::Selectable("ROTATE", operationMode == "ROTATE")){
        //         operationMode = "ROTATE";
        //         modelControlNode->set_gizmo_operation("ROTATE");
        //     }

        //     ImGui::EndCombo();
        // }

        auto viewer = guik::viewer();
        viewer->register_ui_callback(drawableName, [=] {
            modelControlNode->draw_gizmo();
            // //here we update the transform matrix element by element
            // transform->row(0) = modelControlNode->model_matrix().row(0);
            // transform->row(1) = modelControlNode->model_matrix().row(1);
            // transform->row(2) = modelControlNode->model_matrix().row(2);
        });



        ImGui::EndGroup();

        return true;
    }

    static void guizmoPanel(std::string drawableName, std::shared_ptr<guik::ModelControl> modelControlNode, std::string& operationMode){
        ImGui::BeginGroup();
        ImGui::Text("Control Mode");
        ImGui::SameLine();

        //here we want to make a dropdown menu to select the operation mode
        if (ImGui::BeginCombo("##operationMode", operationMode.c_str())){
            if (ImGui::Selectable("TRANSLATE", operationMode == "TRANSLATE")){
                operationMode = "TRANSLATE";
                modelControlNode->set_gizmo_operation("TRANSLATE");
            }
            if (ImGui::Selectable("ROTATE", operationMode == "ROTATE")){
                operationMode = "ROTATE";
                modelControlNode->set_gizmo_operation("ROTATE");
            }

            ImGui::EndCombo();
        }

    }

    static void turnOffGuizmo(std::shared_ptr<guik::ModelControl> modelControlNode, std::string drawableName){
        modelControlNode->disable_gizmo();
        auto viewer = guik::viewer();
        viewer->remove_ui_callback(drawableName);
    }



}










#endif // GUI_SYNTAX_GUIZMOCONTROL_HPP