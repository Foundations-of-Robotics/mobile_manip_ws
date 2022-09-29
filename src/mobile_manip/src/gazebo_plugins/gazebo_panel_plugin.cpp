/*
 * Copyright (c) 2016, JSK Robotics Laboratory, The University of Tokyo
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <gazebo_plugins/gazebo_panel_plugin.h>

namespace gazebo {

GazeboPanel::GazeboPanel()
{
}

GazeboPanel::~GazeboPanel()
{
  //event::Events::DisconnectWorldUpdateBegin(update_connection_);
  update_connection_.reset();

  node_handle_->shutdown();
  delete node_handle_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboPanel::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  gzwarn << "The GazeboPanel plugin is DEPRECATED in ROS hydro." << std::endl;

  model_ = _model;
  world_ = _model->GetWorld();
  link_ = _model->GetLink();
  link_name_ = link_->GetName();
  namespace_.clear();

  terminated_ = false;

  // load parameters from sdf
  if (_sdf->HasElement("robotNamespace")) namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();

  if (_sdf->HasElement("bodyName") && _sdf->GetElement("bodyName")->GetValue()) {
    link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();
    link_ = _model->GetLink(link_name_);
  }

  if (_sdf->HasElement("jointName") && _sdf->GetElement("jointName")->GetValue()) {
    std::string joint_name_ = _sdf->GetElement("jointName")->Get<std::string>();
    joint_ = _model->GetJoint(joint_name_);
  } else {
    joint_ = _model->GetJoint("stem_joint");
  }

  if (!link)
  {
    ROS_FATAL("gazebo_ros_baro plugin error: bodyName: %s does not exist\n", link_name_.c_str());
    return;
  }


  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                     << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  node_handle_ = new ros::NodeHandle(namespace_);
  pub_score_ = node_handle_->advertise<std_msgs::String>("score", 1, true); // set latch true
  pub_time_ = node_handle_->advertise<std_msgs::String>("remaining_time", 1);
  ros::NodeHandle param_handle(*node_handle_, "controller");



  update_connection_ = event::Events::ConnectWorldUpdateBegin(
                                                              boost::bind(&GazeboPanel::Update, this));
}


////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboPanel::Update()
{

  boost::mutex::scoped_lock scoped_lock(lock);

  if ( terminated_ ) {
    return;
  }

  //common::Time current_time = world_->GetSimTime();
  common::Time current_time = world_->SimTime();

  // check score
  //double angle = joint_->GetAngle(0).Radian();
  double angle = joint_->Position(0);
  if ( fmod(current_time.Double(), 1) == 0 ) {
    ROS_INFO_STREAM("Current time is " << current_time.Double() << ", Current angle is = " << angle);
  }

  // void Entity::GetNearestEntityBelow(double &_distBelow,  std::string &_entityName)
  //
  std::stringstream ss;
  std_msgs::String msg_time;
  ss << 20*60 - current_time.Double();
  msg_time.data = ss.str();
  pub_time_.publish(msg_time);
  if ( fabs(angle) > 3.14) {
    std_msgs::String msg_score, msg_time;
    msg_score.data = "Mission Completed";
    ROS_INFO_STREAM("Remaining time is " << msg_time.data << "[sec], Score is " << msg_score.data);
    pub_score_.publish(msg_score);
    terminated_ = true;
  }
}

////////////////////////////////////////////////////////////////////////////////
// Reset the controller
void GazeboPanel::Reset()
{
  state_stamp_ = ros::Time();
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboPanel)

} // namespace gazebo
