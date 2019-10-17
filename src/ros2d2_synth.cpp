/*==========================================================================
  ros2d2_synth.cpp
  Copyright (c)2018 Kenji Koide
  Distributed under the terms of the GNU Public Licence, V3.0
==========================================================================*/

#include <memory>
#include <iostream>
#include <boost/algorithm/string.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>

#define main r2d2_voice_main
#define srand(a) srand(0)
    extern "C" {
#include <r2d2_voice.c>
}
#undef srand
#undef main


class R2D2SynthNode {
public:
  R2D2SynthNode()
    : private_nh("~"),
      cmd_sub(private_nh.subscribe("synth_cmd", 10, &R2D2SynthNode::cmd_callback, this))
  {
  }

private:
  void cmd_callback(const std_msgs::StringConstPtr& cmd_msg) {
    std::vector<std::string> tokens;
    boost::split(tokens, cmd_msg->data, boost::is_any_of(" "));

    std::vector<char*> argv(tokens.size() + 1);
    argv[0] = nullptr;
    for(int i=0; i<tokens.size(); i++) {
      argv[i+1] = &tokens[i][0];
    }

    if(r2d2_voice_main(argv.size(), &argv[0]) < 0) {
      ROS_INFO_STREAM("failed to play sound");
      ROS_INFO_STREAM("cmd = " << cmd_msg->data);
    }
  }

private:
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  ros::Subscriber cmd_sub;
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "ros2d2_synth");

  std::unique_ptr<R2D2SynthNode> node(new R2D2SynthNode());
  ros::spin();

  return 0;
}
