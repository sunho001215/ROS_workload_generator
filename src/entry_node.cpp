#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros/this_node.h"

#define MAGIC_NUMBER 497000

static int child_num_;
static float default_waste_time_ = 1000.0;
static float period_ = 100.0;
static int publish_topic_count_ = 0;

static std::vector<int> child_idx_;
static std::vector<ros::Publisher> publisher_list_;

void parameter_init(ros::NodeHandle nh){
    std::string node_name = ros::this_node::getName().c_str();
    std::string child_idx_param_name = node_name + "/child_idx";
    std::string default_waste_time_param_name = node_name + "/default_waste_time";
    std::string period_param_name = node_name + "/period";

    nh.getParam(child_idx_param_name, child_idx_);
    nh.getParam(default_waste_time_param_name, default_waste_time_);
    nh.getParam(period_param_name, period_);

    child_num_ = child_idx_.size();
}

void publisher_init(ros::NodeHandle nh){
    std::string node_name = ros::this_node::getName().c_str();

    for(int i=0; i<child_num_; i++){
        std::string topic_name = "topic_" + node_name.substr(1) + "_node" + std::to_string(child_idx_.at(i));
        ros::Publisher pub = nh.advertise<std_msgs::String>(topic_name, 10);
        publisher_list_.push_back(pub);
    }
}

void publish_message(){
    std_msgs::String msg;
    std::stringstream ss;
    ss << publish_topic_count_;
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

    publish_message();
    publish_topic_count_++;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "entry_node");
    ros::NodeHandle nh;

    parameter_init(nh);
    publisher_init(nh);

    ros::Rate loop_rate(1000 / period_);

    publish_topic_count_ = 0;
    while (ros::ok()){
        default_waste_time();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}