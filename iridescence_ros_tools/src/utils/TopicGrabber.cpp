#include <utils/TopicGrabber.hpp>


namespace irtools {

    TopicGrabber::TopicGrabber(ros::NodeHandle& nh) : nh_(nh) {}

    TopicGrabber::~TopicGrabber() {}

    bool TopicGrabber::addNewTopic(std::string parentField, std::string topicName) {
        if (topicNodes_.find(parentField) != topicNodes_.end()) {
            topicNodes_[parentField].topicNames.push_back(topicName);
            return true;
        }
        else {
            topicNode newTopicNode;
            newTopicNode.parentField = parentField;
            newTopicNode.topicNames.push_back(topicName);
            topicNodes_[parentField] = newTopicNode;
            return true;
        }
    }

    void TopicGrabber::updateTopicList() {
        //clear the topic node map first
        topicNodes_.clear();
        ros::master::V_TopicInfo master_topics;
        ros::master::getTopics(master_topics);
        // int topicCount = 0;
        for (ros::master::V_TopicInfo::iterator it = master_topics.begin(); it != master_topics.end(); it++) {
            std::string topic = it->name;
            //the parent field should be the context before the last "/"
            std::string parentField = topic.substr(0, topic.find_last_of("/"));
            std::string topicName = topic.substr(topic.find_last_of("/") + 1);

            addNewTopic(parentField, topicName);
            // topicCount++;
            // printf("Topic name: %s\n", topic.c_str());
        }

        // printf("Number of topicNodes: %lu\n", topicNodes_.size());
        // std::cout << "Topic list updated" << std::endl;
        // printf("Number of topics: %d\n", topicCount);
    }






} // namespace irtools
