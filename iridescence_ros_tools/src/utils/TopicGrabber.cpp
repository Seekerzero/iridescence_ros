#include <utils/TopicGrabber.hpp>


namespace irtools {

    TopicGrabber::TopicGrabber(ros::NodeHandle& nh) : nh_(nh) {}

    TopicGrabber::~TopicGrabber() {}

    void TopicGrabber::updateTopicList() {
        topics_.clear();
        ros::master::V_TopicInfo master_topics;
        ros::master::getTopics(master_topics);
        for (const auto& topic : master_topics) {
            topics_.push_back(topic.name);
        }
    }






} // namespace irtools
