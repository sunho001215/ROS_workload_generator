#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros/this_node.h"
#include <ros/package.h>
#include <pthread.h>
#include <unistd.h>

#include <sstream>
#include <vector>
#include <string>
#include <time.h>


#define MAGIC_NUMBER 497000

static int child_num_, parent_num_;
static int msg_count_, iter_ = 0, pid_;
static float callback_waste_time_ = 1000.0;
static float default_waste_time_ = 1000.0;
static float period_ = 100.0;

std::vector<ros::Subscriber> subscriber_list_;
std::vector<int> parent_idx_;
std::vector<bool> sub_flag_;

void parameter_init(ros::NodeHandle nh){
    std::string node_name = ros::this_node::getName().c_str();
    std::string parent_idx_param_name = node_name + "/parent_idx";
    std::string callback_waste_time_param_name = node_name + "/callback_waste_time";
    std::string default_waste_time_param_name = node_name + "/default_waste_time";
    std::string period_param_name = node_name + "/period";

    nh.getParam(parent_idx_param_name, parent_idx_);
    nh.getParam(callback_waste_time_param_name, callback_waste_time_);
    nh.getParam(default_waste_time_param_name, default_waste_time_);
    nh.getParam(period_param_name, period_);

    parent_num_ = parent_idx_.size();

    for(int i=0; i<parent_num_; i++){
      sub_flag_.push_back(false);
    }
}

void default_waste_time(){
    float tmp = 3.323;
    for(long long int i = 0; i < MAGIC_NUMBER * default_waste_time_; i++){
        tmp = 1 - tmp;
    }

    iter_++;
}

void callback_waste_time(){
    struct timespec start_time, end_time;
    clock_gettime(CLOCK_MONOTONIC, &start_time);

    float tmp = 3.323;
    for(long long int i = 0; i < MAGIC_NUMBER * callback_waste_time_; i++){
        tmp = 1 - tmp;
    }

    clock_gettime(CLOCK_MONOTONIC, &end_time);

    std::string pack_path = ros::package::getPath("ros_workload_generator");
    std::string file_name = pack_path + "/csv/leaf_node.csv";
    
    FILE *fp = fopen(file_name.c_str(), "a");
    fprintf(fp, "%d,%d,%ld.%.9ld,%ld.%.9ld,%d,%d\n", iter_, pid_, start_time.tv_sec, start_time.tv_nsec, end_time.tv_sec, end_time.tv_nsec, msg_count_, 1);
    fclose(fp);
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

void reset_file(){
    std::string pack_path = ros::package::getPath("ros_workload_generator");
    std::string file_name = pack_path + "/csv/leaf_node.csv";
    FILE *fp = fopen(file_name.c_str(), "w");
    fprintf(fp, "iter,PID,start,end,instance,activation\n");
    fclose(fp);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "leaf_node");
    
    ros::NodeHandle nh;

    parameter_init(nh);
    subscriber_init(nh);

    ros::Rate loop_rate(1000 / period_);

    reset_file();

    iter_ = 0;
    pid_ = getpid();
    while (ros::ok()){
        ros::spinOnce();
        default_waste_time();
        loop_rate.sleep();
    }

    return 0;
}