#ifndef TOPICGRABBER_HPP
#define TOPICGRABBER_HPP

#include <ros/ros.h>
#include <stdio.h>
#include <unordered_map>

namespace irtools {

struct topicNode{
  std::string parentField;
  std::vector<std::string> topicNames;
};

class TopicGrabber {
  private:
    ros::NodeHandle& nh_;

    //here we customize a structure for storing the topic name and its parent field
    std::unordered_map<std::string, topicNode> topicNodes_;
    // std::vector<std::string> topics_;

    bool addNewTopic(std::string topicName, std::string parentField);
    


  public:
    TopicGrabber(ros::NodeHandle& nh);
    virtual ~TopicGrabber();
    using Ptr = std::shared_ptr<TopicGrabber>;
    void updateTopicList();

    void spin() {
      ros::spin();
    }

    std::vector<std::string> getTopics() {
      updateTopicList();
      std::vector<std::string> topics;
      for (auto it = topicNodes_.begin(); it != topicNodes_.end(); it++) {
        std::string parentField = it->first;
        for (auto topic : it->second.topicNames) {
          topics.push_back(parentField + "/" + topic);
        }
      }
      // printf("Number of topics: %lu\n", topics.size());
      return topics;
    }

};

} // namespace irtools


#endif // TOPICGRABBER_HPP