#ifndef GUI_CONTROLLERS_ITOOLCONTROLLER_HPP
#define GUI_CONTROLLERS_ITOOLCONTROLLER_HPP

#include <memory>

namespace gui{

    //The tool controller is used for convert ui input to the actual operation on the model
    class IToolController{
        public:
            IToolController(){}
            virtual ~IToolController(){}
            using Ptr = std::shared_ptr<IToolController>;


        private:
            // Prevent copying
            IToolController(const IToolController&) = delete;
            IToolController& operator=(const IToolController&) = delete;
    };
}







#endif // GUI_CONTROLLERS_ITOOLCONTROLLER_HPP