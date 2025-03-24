
#include <ros/ros.h>
#include <viewer/viewer.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "viewer");
  irtviewer::Viewer viewer;
  

  return 0;
}