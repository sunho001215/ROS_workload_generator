#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros/this_node.h"
#include <ros/package.h>
#include <pthread.h>

#include <sstream>
#include <vector>
#include <string>
#include <time.h>

#define MAGIC_NUMBER 497000

static int child_num_, parent_num_;
static int msg_count_;
static float callback_waste_time_ = 1000.0;
static float default_waste_time_ = 1000.0;
static float period_ = 100.0;

std::vector<ros::Publisher> publisher_list_;
std::vector<ros::Subscriber> subscriber_list_;
std::vector<int> parent_idx_, child_idx_;
std::vector<bool> sub_flag_;

void parameter_init(ros::NodeHandle nh){
    std::string node_name = ros::this_node::getName().c_str();
    std::string child_idx_param_name = node_name + "/child_idx";
    std::string parent_idx_param_name = node_name + "/parent_idx";
    std::string callback_waste_time_param_name = node_name + "/callback_waste_time";
    std::string default_waste_time_param_name = node_name + "/default_waste_time";
    std::string period_param_name = node_name + "/period";

    nh.getParam(child_idx_param_name, child_idx_);
    nh.getParam(parent_idx_param_name, parent_idx_);
    nh.getParam(callback_waste_time_param_name, callback_waste_time_);
    nh.getParam(default_waste_time_param_name, default_waste_time_);
    nh.getParam(period_param_name, period_);

    parent_num_ = parent_idx_.size();
    child_num_ = child_idx_.size();

    for(int i=0; i<parent_num_; i++){
      sub_flag_.push_back(false);
    }
}

void publisher_init(ros::NodeHandle nh){
    std::string node_name = ros::this_node::getName().c_str();

    for(int i=0; i<child_num_; i++){
        std::string topic_name = "topic_" + node_name.substr(1) + "_node" + std::to_string(child_idx_.at(i));
        ros::Publisher pub = nh.advertise<std_msgs::String>(topic_name, 1);
        publisher_list_.push_back(pub);
    }
}

void publish_message(){
    std_msgs::String msg;
    std::stringstream ss;
    ss << msg_count_;
    msg.data = ss.str();

    for(int i=0; i<child_num_; i++){
        publisher_list_[i].publish(msg);
    }
}

void default_waste_time(){
    float tmp = 3.323;
    for(long long int i = 0; i < MAGIC_NUMBER * default_waste_time_; i++){
        tmp = 1 - tmp;
    }
}

void callback_waste_time(){
    float tmp = 3.323;
    for(long long int i = 0; i < MAGIC_NUMBER * callback_waste_time_; i++){
        tmp = 1 - tmp;
    }

    publish_message();
}

void topic_callback(const std_msgs::String::ConstPtr& msg, int topic_idx)
{
  sub_flag_.at(topic_idx) = true;

  if(find(sub_flag_.begin(), sub_flag_.end(), false) == sub_flag_.end()){
    for(int i=0; i<parent_num_; i++){
      sub_flag_.at(i) = false;
    }

    msg_count_ = std::stoi(msg->data.c_str());
    callback_waste_time();
  }
}

void subscriber_init(ros::NodeHandle nh){
  std::string node_name = ros::this_node::getName().c_str();

  for(int i=0; i<parent_num_; i++){
      std::string topic_name = "topic_node" + std::to_string(parent_idx_.at(i)) + "_" + node_name.substr(1);

      ros::Subscriber sub = nh.subscribe<std_msgs::String>(topic_name, 1, boost::bind(topic_callback, _1, i));

      subscriber_list_.push_back(sub);
  }
}

int main(int argc, char **argv){
    ros::init(argc, argv ,"node4");
    
    ros::NodeHandle nh;

    parameter_init(nh);
    publisher_init(nh);
    subscriber_init(nh);

    ros::Rate loop_rate(1000 / period_);

    while (ros::ok()){
        default_waste_time();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}