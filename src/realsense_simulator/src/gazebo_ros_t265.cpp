/*
 * Desc: 3D odometry publisher for simulating a Realsense T265 tacking camera - based on P3D plugin by Sachin Chitta and John Hsu
 * Author: Rafael Gomes Braga
 * Date: 23 October 2020
 */

#include <string>
#include <tf/tf.h>
#include <stdlib.h>

#include "realsense_gazebo_plugin/gazebo_ros_t265.h"

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(GazeboRosT265);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosT265::GazeboRosT265()
{
  this->seed = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosT265::~GazeboRosT265()
{
  this->update_connection_.reset();
  // Finalize the controller
  this->rosnode_->shutdown();
  this->queue_.clear();
  this->queue_.disable();
  this->callback_queue_thread_.join();
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosT265::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // Get the world name.
  this->world_ = _parent->GetWorld();
  this->model_ = _parent;

  // Store the model's initial pose
  this->init_pose_ = this->model_->WorldPose();
  ROS_WARN_NAMED("t265", "T265 INIT pose--------%f, %f, %f", this->init_pose_.Pos().X(), this->init_pose_.Pos().Y(), this->init_pose_.Pos().Z());
  // load parameters
  this->robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace_ =
      _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  if (!_sdf->HasElement("cameraName"))
  {
    ROS_FATAL_NAMED("t265", "t265 plugin missing <cameraName>, cannot proceed");
    return;
  }
  else
    this->camera_name_ = _sdf->GetElement("cameraName")->Get<std::string>();

  if (!_sdf->HasElement("tfPrefix"))
  {
    this->tf_prefix_ = "";
    ROS_DEBUG_NAMED("t265", "t265 plugin missing <tfPrefix>, defaults to %s",
      this->tf_prefix_.c_str());
  }
  else
    this->tf_prefix_ = _sdf->GetElement("tfPrefix")->Get<std::string>();

  if (!_sdf->HasElement("odomFrameId"))
  {
    ROS_DEBUG_NAMED("t265", "t265 plugin missing <odomFrameId>, defaults to %s_odom_frame",
      this->camera_name_.c_str());
    this->odom_frame_id_ = this->camera_name_ + "_odom_frame";
  }
  else
    this->odom_frame_id_ = _sdf->GetElement("odomFrameId")->Get<std::string>();

  if (!_sdf->HasElement("poseFrameId"))
  {
    ROS_DEBUG_NAMED("t265", "t265 plugin missing <poseFrameId>, defaults to %s_pose_frame",
      this->camera_name_.c_str());
    this->pose_frame_id_ = this->camera_name_ + "_pose_frame";
  }
  else
    this->pose_frame_id_ = _sdf->GetElement("poseFrameId")->Get<std::string>();

  // Add tf prefix to frame names
  if (!this->tf_prefix_.empty())
  {
    this->odom_frame_id_ = this->tf_prefix_ + "/" + this->odom_frame_id_;
    this->pose_frame_id_ = this->tf_prefix_ + "/" + this->pose_frame_id_;
  }

  if (!_sdf->HasElement("xyzOffsets"))
  {
    ROS_DEBUG_NAMED("t265", "t265 plugin missing <xyzOffsets>, defaults to 0s");
    this->offset_.Pos() = ignition::math::Vector3d(0, 0, 0);
  }
  else {
    this->offset_.Pos() = _sdf->GetElement("xyzOffsets")->Get<ignition::math::Vector3d>();
    ROS_WARN_NAMED("t265", "T265 INIT pose offsets--------%f, %f, %f", this->offset_.Pos().X(), this->offset_.Pos().Y(), this->offset_.Pos().Z());
  }

  if (!_sdf->HasElement("rpyOffsets"))
  {
    ROS_DEBUG_NAMED("t265", "t265 plugin missing <rpyOffsets>, defaults to 0s");
    this->offset_.Rot() = ignition::math::Quaterniond(ignition::math::Vector3d(0, 0, 0));
  }
  else
    this->offset_.Rot() = ignition::math::Quaterniond(_sdf->GetElement("rpyOffsets")->Get<ignition::math::Vector3d>());

  if (!_sdf->HasElement("gaussianNoise"))
  {
    ROS_DEBUG_NAMED("t265", "t265 plugin missing <gaussianNoise>, defaults to 0.0");
    this->gaussian_noise_ = 0;
  }
  else
    this->gaussian_noise_ = _sdf->GetElement("gaussianNoise")->Get<double>();

  if (!_sdf->HasElement("updateRate"))
  {
    ROS_DEBUG_NAMED("t265", "t265 plugin missing <updateRate>, defaults to 0.0"
             " (as fast as possible)");
    this->update_rate_ = 0;
  }
  else
    this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("t265", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  // publish multi queue
  this->pmq.startServiceThread();

  // Setup odometry publisher
  this->topic_name_ = this->camera_name_ + "/odom/sample";
  this->pub_Queue = this->pmq.addPub<nav_msgs::Odometry>();
  this->pub_ = this->rosnode_->advertise<nav_msgs::Odometry>(this->topic_name_, 1);

#if GAZEBO_MAJOR_VERSION >= 8
  this->last_time_ = this->world_->SimTime();
#else
  this->last_time_ = this->world_->GetSimTime();
#endif

  // Setup tf broadcaster
  odom_broadcaster_ = boost::shared_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());

  // start custom queue for t265
  this->callback_queue_thread_ = boost::thread(
    boost::bind(&GazeboRosT265::QueueThread, this));

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboRosT265::UpdateChild, this));
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosT265::UpdateChild()
{

#if GAZEBO_MAJOR_VERSION >= 8
  common::Time cur_time = this->world_->SimTime();
#else
  common::Time cur_time = this->world_->GetSimTime();
#endif

  if (cur_time < this->last_time_)
  {
      ROS_WARN_NAMED("t265", "Negative update time difference detected.");
      this->last_time_ = cur_time;
  }

  // rate control
  if (this->update_rate_ > 0 &&
      (cur_time-this->last_time_).Double() < (1.0/this->update_rate_))
    return;

  //if (this->pub_.getNumSubscribers() > 0)
  //{
  //  this->lock.lock();

  ignition::math::Pose3d pose;

  // get inertial Rates
  // Get Pose/Orientation
#if GAZEBO_MAJOR_VERSION >= 8
  ignition::math::Vector3d vpos = this->model_->WorldLinearVel();
  ignition::math::Vector3d veul = this->model_->WorldAngularVel();

  pose = this->model_->WorldPose();
#else
  ignition::math::Vector3d vpos = this->model_->GetWorldLinearVel().Ign();
  ignition::math::Vector3d veul = this->model_->GetWorldAngularVel().Ign();

  pose = this->model_->GetWorldPose().Ign();
#endif

  // Apply initial pose
  pose.Pos() = pose.Pos() - init_pose_.Pos();
  pose.Pos() = init_pose_.Rot().RotateVectorReverse(pose.Pos());
  pose.Rot() *= init_pose_.Rot().Inverse();

  // Apply Constant Offsets
  // apply xyz offsets and get position and rotation components
  pose.Pos() = pose.Pos() + this->offset_.Pos();
  // apply rpy offsets
  pose.Rot() = this->offset_.Rot()*pose.Rot();
  pose.Rot().Normalize();

  // Store pose in vector and quaternion formats for later use
  tf::Quaternion qt;
  tf::Vector3 vt;

  qt = tf::Quaternion(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W());
  vt = tf::Vector3(
    pose.Pos().X() + this->GaussianKernel(0, this->gaussian_noise_),
    pose.Pos().Y() + this->GaussianKernel(0, this->gaussian_noise_),
    pose.Pos().Z() + this->GaussianKernel(0, this->gaussian_noise_)
  );

  // Publish the transform over tf
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.frame_id = this->odom_frame_id_;
  odom_trans.child_frame_id = this->pose_frame_id_;
  odom_trans.header.stamp.sec = cur_time.sec;
  odom_trans.header.stamp.nsec = cur_time.nsec;

  odom_trans.transform.translation.x = vt.x();
  odom_trans.transform.translation.y = vt.y();
  odom_trans.transform.translation.z = vt.z();

  odom_trans.transform.rotation.x = qt.x();
  odom_trans.transform.rotation.y = qt.y();
  odom_trans.transform.rotation.z = qt.z();
  odom_trans.transform.rotation.w = qt.w();

  //send the transform
  this->odom_broadcaster_->sendTransform(odom_trans);

  // tf::Transform base_footprint_to_odom(qt, vt);
  // this->odom_broadcaster_.sendTransform(
  //   tf::StampedTransform(base_footprint_to_odom, current_time,
  //                        odom_frame, base_frame));

  // copy data into pose message
  this->pose_msg_.header.frame_id = this->odom_frame_id_;
  this->pose_msg_.child_frame_id = this->pose_frame_id_;
  this->pose_msg_.header.stamp.sec = cur_time.sec;
  this->pose_msg_.header.stamp.nsec = cur_time.nsec;

  // Fill out messages
  this->pose_msg_.pose.pose.position.x = vt.x();
  this->pose_msg_.pose.pose.position.y = vt.y();
  this->pose_msg_.pose.pose.position.z = vt.z();

  this->pose_msg_.pose.pose.orientation.x = qt.x();
  this->pose_msg_.pose.pose.orientation.y = qt.y();
  this->pose_msg_.pose.pose.orientation.z = qt.z();
  this->pose_msg_.pose.pose.orientation.w = qt.w();

  this->pose_msg_.twist.twist.linear.x  = vpos.X() +
    this->GaussianKernel(0, this->gaussian_noise_);
  this->pose_msg_.twist.twist.linear.y  = vpos.Y() +
    this->GaussianKernel(0, this->gaussian_noise_);
  this->pose_msg_.twist.twist.linear.z  = vpos.Z() +
    this->GaussianKernel(0, this->gaussian_noise_);
  // pass euler angular rates
  this->pose_msg_.twist.twist.angular.x = veul.X() +
    this->GaussianKernel(0, this->gaussian_noise_);
  this->pose_msg_.twist.twist.angular.y = veul.Y() +
    this->GaussianKernel(0, this->gaussian_noise_);
  this->pose_msg_.twist.twist.angular.z = veul.Z() +
    this->GaussianKernel(0, this->gaussian_noise_);

  // fill in covariance matrix
  /// @todo: let user set separate linear and angular covariance values.
  double gn2 = this->gaussian_noise_*this->gaussian_noise_;
  this->pose_msg_.pose.covariance[0] = gn2;
  this->pose_msg_.pose.covariance[7] = gn2;
  this->pose_msg_.pose.covariance[14] = gn2;
  this->pose_msg_.pose.covariance[21] = gn2;
  this->pose_msg_.pose.covariance[28] = gn2;
  this->pose_msg_.pose.covariance[35] = gn2;

  this->pose_msg_.twist.covariance[0] = gn2;
  this->pose_msg_.twist.covariance[7] = gn2;
  this->pose_msg_.twist.covariance[14] = gn2;
  this->pose_msg_.twist.covariance[21] = gn2;
  this->pose_msg_.twist.covariance[28] = gn2;
  this->pose_msg_.twist.covariance[35] = gn2;

  // publish to ros
  this->pub_Queue->push(this->pose_msg_, this->pub_);

    //this->lock.unlock();

  // save last time stamp
  this->last_time_ = cur_time;
  //}
}

//////////////////////////////////////////////////////////////////////////////
// Utility for adding noise
double GazeboRosT265::GaussianKernel(double mu, double sigma)
{
  // using Box-Muller transform to generate two independent standard
  // normally disbributed normal variables see wikipedia

  // normalized uniform random variable
  double U = static_cast<double>(rand_r(&this->seed)) /
             static_cast<double>(RAND_MAX);

  // normalized uniform random variable
  double V = static_cast<double>(rand_r(&this->seed)) /
             static_cast<double>(RAND_MAX);

  double X = sqrt(-2.0 * ::log(U)) * cos(2.0*M_PI * V);
  // double Y = sqrt(-2.0 * ::log(U)) * sin(2.0*M_PI * V);

  // there are 2 indep. vars, we'll just use X
  // scale to our mu and sigma
  X = sigma * X + mu;
  return X;
}

////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void GazeboRosT265::QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}
}
