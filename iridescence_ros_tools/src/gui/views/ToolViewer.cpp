#include <gui/views/ToolViewer.hpp>


namespace irtgui{

    void ToolViewer::defaultUI(){
        //here we make a dropdown list of topics
        if (ImGui::Button("Add A Topic")){
            if (!show_add_topic_window){
                show_add_topic_window = true;
            }
        }

        if (ImGui::Button("Close")) {
            viewer_->close();
        }

        if (show_add_topic_window){
            addTopicWindow();
        }
    }

    void ToolViewer::addTopicWindow(){
        ImGui::Begin("Add Topic", &show_add_topic_window, ImGuiWindowFlags_AlwaysAutoResize);

        ImGui::Text("Select a topic to add to the viewer");

        ImGui::NewLine();

        updateTopicList();

        if (ImGui::BeginCombo("##ToolViewerMenu", drop_list_selected_topic.c_str())) {
            for (int n = 0; n < available_topics_.size(); n++) {
                const bool is_selected = (drop_list_selected_topic == available_topics_[n]);
                if (ImGui::Selectable(available_topics_[n].c_str(), is_selected)) {
                    drop_list_selected_topic = available_topics_[n];
                }
                if (is_selected) {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();
        }
        ImGui::SameLine();
        
        if (drop_list_selected_topic != "Select a topic") {
            if (ImGui::Button("Add")) {
                //add the topic to the viewer
                //subs_.push_back(nh_.subscribe(drop_list_selected_topic, 1, &ToolViewer::callback, this));
                subscribed_topics_.push_back(drop_list_selected_topic);
                drop_list_selected_topic = "Select a topic";
                show_add_topic_window = false;
            }
        }

        ImGui::End();
    
    }

    void ToolViewer::updateSubscribeDrawables(){

    }
}