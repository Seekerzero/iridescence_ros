#pragma once

#include <glk/pointcloud_buffer.hpp>
#include <glk/thin_lines.hpp>
#include <glk/indexed_pointcloud_buffer.hpp>
#include <glk/primitives/primitives.hpp>
#include <glk/splatting.hpp>
#include <guik/viewer/light_viewer.hpp>
#include <guik/camera/fps_camera_control.hpp>
#include <guik/camera/topdown_camera_control.hpp>
#include <guik/hovered_drawings.hpp>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/ringbuffer_sink.h>

#include <guik/spdlog_sink.hpp>
#include <gui/fonts/fonts.hpp>
#include <gui/views/MainMenuPanel.hpp>


namespace irtviewer{
    class Viewer{
        public:
            Viewer(){
                auto viewer = guik::viewer();
                auto &io = ImGui::GetIO();

                io.Fonts->AddFontDefault();

                float baseFontSize = 17.0f;
                float iconFontSize = baseFontSize * 2.0f / 3.0f * 1.2f;
                // merge in icons from google material design
                static const ImWchar icons_ranges2[] = {ICON_MIN_FA, ICON_MAX_16_FA, 0};
                ImFontConfig icons_config;
                icons_config.MergeMode = true;
                icons_config.PixelSnapH = true;
                icons_config.GlyphOffset.x = 0.0f;
                icons_config.GlyphOffset.y = 1.0f;
    
                // icons_config.GlyphMinAdvanceX = iconFontSize; // Use if you want to make the icon monospaced
    
                // ImFont *font2 = io.Fonts->AddFontFromFileTTF(FONT_ICON_FILE_NAME_FAS, iconFontSize, &icons_config, icons_ranges2);
    
                icons_config.GlyphOffset.x = 0.0f;
                icons_config.GlyphOffset.y = 4.0f;
                float iconFontSize2 = baseFontSize * 2.0f / 3.0f * 1.5f;
                static const ImWchar icons_ranges3[] = {ICON_MIN_MS, ICON_MAX_16_MS, 0};
                ImFont *font3 = io.Fonts->AddFontFromFileTTF(FONT_ICON_FILE_NAME_MSR, iconFontSize2, &icons_config, icons_ranges3);
    
                io.Fonts->Build();

                viewer->set_draw_xy_grid(false);

                viewer->register_ui_callback("ui", [this]
                    { ui_callback(); });

                viewer->spin();
            }

            ~Viewer(){
                auto viewer = guik::viewer();
                viewer->remove_ui_callback("ui");
            }
            static std::shared_ptr<Viewer> instance;

            using Ptr = std::shared_ptr<Viewer>;

            static Ptr getInstance()
            {
                if (!instance)
                {
                    instance = std::make_shared<Viewer>();
                }
                return instance;
            }
            

        private:
            void ui_callback()
            {
                this->main_menu_panel_->registerToolUI();
            }
        
        private:
            std::shared_ptr<irtgui::MainMenuPanel> main_menu_panel_ = std::make_shared<irtgui::MainMenuPanel>("Main Menu", 300, 600, "MainMenuPanel", true);
    };
}