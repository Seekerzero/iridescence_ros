#ifndef TOPICGRABBER_HPP
#define TOPICGRABBER_HPP

#include <ros/ros.h>
#include <stdio.h>

namespace irtools {

class TopicGrabber {
public:
  TopicGrabber(ros::NodeHandle& nh) : nh_(nh) {}
//   template <typename T>
//   void subscribe(const std::string& topic, uint32_t queue_size, const std::function<void(const T&)>& callback) {
//     ros::Subscriber sub = nh_.subscribe<T>(topic, queue_size, callback);
//     subs_.push_back(sub);
//   }

  void updateTopicList() {
    topics_.clear();
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);
    for (const auto& topic : master_topics) {
      topics_.push_back(topic.name);
    }
  }

  void spin() {
    ros::spin();
  }

private:
    ros::NodeHandle& nh_;
    // std::vector<ros::Subscriber> subs_;
    std::vector<std::string> topics_;


};

} // namespace irtools


#endif // TOPICGRABBER_HPP