#ifndef TOPICGRABBER_HPP
#define TOPICGRABBER_HPP

#include <ros/ros.h>
#include <stdio.h>

namespace irtools {

class TopicGrabber {
  private:
    ros::NodeHandle& nh_;
    // std::vector<ros::Subscriber> subs_;
    std::vector<std::string> topics_;


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
      return topics_;
    }

};

} // namespace irtools


#endif // TOPICGRABBER_HPP