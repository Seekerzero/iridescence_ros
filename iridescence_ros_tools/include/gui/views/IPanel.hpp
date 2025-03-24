#pragma once

#include <guik/viewer/light_viewer.hpp>
#include <guik/hovered_drawings.hpp>
#include <guik/model_control.hpp>
#include <gui/fonts/fonts.hpp>

namespace irtgui
{
    class IPanel
    {
    public:
        IPanel(std::string name, float width, float height, std::string panelTag, bool open = true) : pName(name), pWidth(width), pHeight(height), pOpen(open), pTag(panelTag) {}
        ~IPanel() {};
        using Ptr = std::shared_ptr<IPanel>;
        std::string pName = "";

        virtual void registerToolUI() = 0;
        virtual void toolUI() = 0;

        float getWidth() const { return pWidth; }
        float getHeight() const { return pHeight; }
        std::string getName() const { return pName; }
        std::string getTag() const { return pTag; }
        bool isOpen() { return pOpen; }
        bool *openFlag() { return &pOpen; }
        void setOpenFlag(bool flag) { pOpen = flag; }

    private:
        // Prevent copying

        float pWidth = 0.0f;
        float pHeight = 0.0f;
        bool pOpen = true;
        std::string pTag = "";

        IPanel() = default; // Add this line

        IPanel(const IPanel &) = delete;
        IPanel &operator=(const IPanel &) = delete;
    };
}
