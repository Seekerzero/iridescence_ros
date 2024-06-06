
#include <ros/ros.h>
#include <gui/ToolViewer.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "viewer");
  irtools::ToolViewer viewer;
  viewer.spin();
  

  return 0;
}