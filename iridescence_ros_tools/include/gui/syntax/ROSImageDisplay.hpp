#pragma once

#include <guik/viewer/light_viewer.hpp>
#include <glk/texture.hpp>
#include <glk/texture_opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>


namespace irtgui
{
    static void updateROSImageDisplay(sensor_msgs::ImageConstPtr image, std::string topicName, guik::LightViewer* viewer)
    {
        // Check if the image is not null and has valid data
        if (image != nullptr && image->data.size() > 0)
        {
            cv::Mat cv_image;
            // Convert the ROS image message to OpenCV format
            cv::Mat cv_image_temp = cv::Mat(image->height, image->width, CV_8UC3, const_cast<uint8_t*>(image->data.data()), image->step);
            //check encoding
            // std::cout << "Image encoding: " << image->encoding << std::endl;
            if (image->encoding == "bgr8")
            {
                //convert to rgb8
                // cv::cvtColor(cv_image_temp, cv_image, cv::COLOR_BGR2RGB);
                cv_image = cv_image_temp.clone();
            }
            else{
                //clone the image
                cv::cvtColor(cv_image_temp, cv_image, cv::COLOR_RGB2BGR);
                // cv_image = cv_image_temp.clone();
            }

            auto texture = glk::create_texture(cv_image);

            viewer->update_image(topicName, texture);
            
        }
        else
        {
            viewer->remove_image(topicName);
            std::cout << "Image is null or empty" << std::endl;
        }
    }

}