#ifndef TOPICGRABBER_HPP
#define TOPICGRABBER_HPP

#include <ros/ros.h>
#include <stdio.h>
#include <unordered_map>

namespace irtutils {

struct topicNode{
  std::string parentField;
  std::vector<std::string> topicNames;
};

struct topicDisplayNode{
  std::string value;
  std::string parentField;
  std::string dataType;
  bool isParent;
  std::unordered_map<std::string, std::shared_ptr<topicDisplayNode>> children;
  // topicDisplayNode(std::string value, bool isParent) : value(value), isParent(isParent) {}

  //constructor
  topicDisplayNode(std::string value, std::string parentField, bool isParent, std::string dataType ="TopicPath") : value(value), parentField(parentField), isParent(isParent), dataType(dataType) {
    // Initialize the children map
    children = std::unordered_map<std::string, std::shared_ptr<topicDisplayNode>>();
  }

  std::shared_ptr<topicDisplayNode> addChild(std::string childName, topicDisplayNode childNode) {
    //check if the childName already exists
    if (children.find(childName) == children.end()) {
      children[childName] = std::make_shared<topicDisplayNode>(childNode);
      return children[childName];
    }
    else {
      //if the childName already exists, return the existing child node
      return children[childName];
    }
  }

  std::shared_ptr<topicDisplayNode> getChild(std::string childName) {
    if (children.find(childName) != children.end()) {
      return children[childName];
    }
    else {
      return nullptr;
    }
  }

};

class TopicGrabber {
  private:
    ros::NodeHandle& nh_;

    //here we customize a structure for storing the topic name and its parent field
    std::unordered_map<std::string, topicNode> topicNodes_;
    std::shared_ptr<topicDisplayNode> rootTree_;
    // std::vector<std::string> topics_;

    bool addNewTopic(std::string topicName, std::string parentField);
    


  public:
    TopicGrabber(ros::NodeHandle& nh);
    virtual ~TopicGrabber();
    using Ptr = std::shared_ptr<TopicGrabber>;
    // void updateTopicList();
    void updateTopicTreeNode();
    // std::vector<std::string> getTopics();
    // topicDisplayNode createTopicDisplayNodeTree();
    std::shared_ptr<topicDisplayNode> getTopicDisplayRoot()
    {
      return rootTree_;
    }

    // void spin() {
    //   ros::spin();
    // }

};

} // namespace irtutils


#endif // TOPICGRABBER_HPP