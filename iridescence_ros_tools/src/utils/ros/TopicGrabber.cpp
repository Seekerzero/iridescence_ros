#include <utils/ros/TopicGrabber.hpp>


namespace irtutils {

    TopicGrabber::TopicGrabber(ros::NodeHandle& nh) : nh_(nh) {}

    TopicGrabber::~TopicGrabber() {}

    // bool TopicGrabber::addNewTopic(std::string parentField, std::string topicName) {
    //     if (topicNodes_.find(parentField) != topicNodes_.end()) {
    //         topicNodes_[parentField].topicNames.push_back(topicName);
    //         return true;
    //     }
    //     else {
    //         topicNode newTopicNode;
    //         newTopicNode.parentField = parentField;
    //         newTopicNode.topicNames.push_back(topicName);
    //         topicNodes_[parentField] = newTopicNode;
    //         return true;
    //     }
    // }

    // void TopicGrabber::updateTopicList() {
    //     //clear the topic node map first
    //     topicNodes_.clear();
    //     ros::master::V_TopicInfo master_topics;
    //     ros::master::getTopics(master_topics);
    //     // int topicCount = 0;
    //     for (ros::master::V_TopicInfo::iterator it = master_topics.begin(); it != master_topics.end(); it++) {
    //         std::string topic = it->name;
    //         //the parent field should be the context before the last "/"
    //         std::string parentField = topic.substr(0, topic.find_last_of("/"));
    //         std::string topicName = topic.substr(topic.find_last_of("/") + 1);

    //         addNewTopic(parentField, topicName);
    //         // topicCount++;
    //         // printf("Topic name: %s\n", topic.c_str());
    //     }

    //     // printf("Number of topicNodes: %lu\n", topicNodes_.size());
    //     // std::cout << "Topic list updated" << std::endl;
    //     // printf("Number of topics: %d\n", topicCount);
    // }


    void TopicGrabber::updateTopicTreeNode(){
        this->rootTree_ = std::make_shared<topicDisplayNode>("", "", true);
        ros::master::V_TopicInfo master_topics;
        ros::master::getTopics(master_topics);

        for (ros::master::V_TopicInfo::iterator it = master_topics.begin(); it != master_topics.end(); it++) {
            std::shared_ptr<topicDisplayNode> currentNode = this->rootTree_;
            std::string rosDataType = it->datatype;
            std::string topic = it->name;
            topic = topic.substr(1, topic.length());
            
            while (topic.find("/") != std::string::npos) {
                std::string subTopicName = topic.substr(0, topic.find_first_of("/"));
                //create a new parent field from current node
                std::string newParentField = currentNode->parentField+currentNode->value+"/";
                currentNode = currentNode->addChild(subTopicName, topicDisplayNode(subTopicName, newParentField, true));
                topic = topic.substr(topic.find_first_of("/") + 1);
            }
            //add the last topic name
            std::string newParentField = currentNode->parentField+currentNode->value+"/";
            currentNode->addChild(topic, topicDisplayNode(topic, newParentField, false, rosDataType));
        }

    }

    // std::vector<std::string> TopicGrabber::getTopics() {
    //     updateTopicList();
    //     std::vector<std::string> topics;
    //     for (auto it = topicNodes_.begin(); it != topicNodes_.end(); it++) {
    //       std::string parentField = it->first;
    //       for (auto topic : it->second.topicNames) {
    //         topics.push_back(parentField + "/" + topic);
    //       }
    //     }
    //     // printf("Number of topics: %lu\n", topics.size());
    //     return topics;
    // }



    
    // topicDisplayNode TopicGrabber::createTopicDisplayNodeTree() {
    //     std::shared_ptr<topicDisplayNode> root = std::make_shared<topicDisplayNode>("root", true);
    //     std::vector<std::string> topics = getTopics();
        
    //     //parse the topics and create the tree using the slash as the delimiter, always ignoring the first element
    //     for (auto topic : topics) {
    //         std::string rootSlach = topic.substr(0, topic.find_first_of("/"));
    //         std::string subTopic = topic.substr(topic.find_first_of("/") + 1);

    //         currentNode = root;
            
    //         while (subTopic.find("/") != std::string::npos) {
    //             std::string subTopicName = subTopic.substr(0, subTopic.find_first_of("/"));
    //             currentNode = currentNode->addChild(subTopicName, topicDisplayNode(subTopicName, true));
    //             subTopic = subTopic.substr(subTopic.find_first_of("/") + 1);    
    //         }
    //     }

    //     return root;
    // }





} // namespace irtools
