/*
 * UOS-ROS packages - Robot Operating System code by the University of Osnabrück
 * Copyright (C) 2014 University of Osnabrück
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in
 * the documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * fps_motion_tool.cpp
 *
 * Author: Henning Deeken {hdeeken@uos.de}
 *
 */

#include <go_home_tool.h>

namespace rviz
{

GoHomeTool::GoHomeTool()
{
  param_property_ = new StringProperty( "Parameter", "home",
                                        "The rosparam which will be used to store the home pose.",
                                        getPropertyContainer(), SLOT( updateParam() ), this );

  topic_property_ = new StringProperty( "Topic", "goal",
                                        "The topic on which to publish navigation goals.",
                                        getPropertyContainer(), SLOT( updateTopic() ), this );
}

GoHomeTool::~GoHomeTool() {}

void GoHomeTool::onInitialize()
{
  setName( "Go Home" );
  updateParam();
  updateTopic();
}


void GoHomeTool::updateTopic()
{
  pub_ = nh_.advertise<geometry_msgs::PoseStamped>( topic_property_->getStdString(), 1 );
}

void GoHomeTool::updateParam()
{
  param_ = param_property_->getStdString();
}

std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}

double fromString(std::string str)
{
    std::istringstream iss(str);
    double d = 0;
    iss >> d;
    return d;
}

void GoHomeTool::activate()
{
  if (ros::param::has(param_))
  {
    std::string pose_string;
    if (ros::param::get(param_, pose_string))
    {
      std::vector<std::string> pose_strings = split(pose_string, ',');

      if(pose_strings.size() == 8)
      {
        geometry_msgs::PoseStamped pose;

        pose.header.frame_id = pose_strings.at(0);
        pose.pose.position.x = fromString(pose_strings.at(1));
        pose.pose.position.y = fromString(pose_strings.at(2));
        pose.pose.position.z = fromString(pose_strings.at(3));
        pose.pose.orientation.x = fromString(pose_strings.at(4));
        pose.pose.orientation.y = fromString(pose_strings.at(5));
        pose.pose.orientation.z = fromString(pose_strings.at(6));
        pose.pose.orientation.w = fromString(pose_strings.at(7));

        pub_.publish(pose);
      }
      else
      {
        ROS_WARN("Parameter is of inconsistent size.");
        return;
      }

      context_->getToolManager()->setCurrentTool(context_->getToolManager()->getDefaultTool());
      return;
    }
  }
  else
  {
    ROS_WARN("No homing pose on parameter %s was set.", param_.c_str());
    return;
  }
}

void GoHomeTool::deactivate(){ }

} // end namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::GoHomeTool, rviz::Tool)
