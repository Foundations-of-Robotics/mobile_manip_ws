/*
 * Desc: 3D odometry publisher for simulating a Realsense T265 tacking camera - based on P3D plugin by Sachin Chitta and John Hsu
 * Author: Rafael Gomes Braga
 * Date: 23 October 2020
 */

#ifndef GAZEBO_ROS_T265_HH
#define GAZEBO_ROS_T265_HH

#include <string>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <tf/transform_broadcaster.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <gazebo_plugins/PubQueue.h>

namespace gazebo
{
  class GazeboRosT265 : public ModelPlugin
  {
    /// \brief Constructor
    public: GazeboRosT265();

    /// \brief Destructor
    public: virtual ~GazeboRosT265();

    /// \brief Load the controller
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Update the controller
    protected: virtual void UpdateChild();

    /// \brief Pointers to relevant models
    private: physics::WorldPtr world_;
    private: physics::ModelPtr model_;

    /// \brief pointer to ros node
    private: ros::NodeHandle* rosnode_;
    private: ros::Publisher pub_;
    private: PubQueue<nav_msgs::Odometry>::Ptr pub_Queue;
    private: boost::shared_ptr<tf::TransformBroadcaster> odom_broadcaster_;

    /// \brief ros message
    private: nav_msgs::Odometry pose_msg_;

    /// \brief topic name
    private: std::string topic_name_;

    /// \brief tf prefix and frame names used by the t265
    private: std::string camera_name_;
    private: std::string tf_prefix_;
    private: std::string odom_frame_id_;
    private: std::string pose_frame_id_;

    /// \brief store initial pose
    private: ignition::math::Pose3d init_pose_;

    /// \brief allow specifying constant xyz and rpy offsets
    private: ignition::math::Pose3d offset_;

    /// \brief mutex to lock access to fields used in message callbacks
    private: boost::mutex lock;

    /// \brief save last_time
    private: common::Time last_time_;

    // rate control
    private: double update_rate_;

    /// \brief Gaussian noise
    private: double gaussian_noise_;

    /// \brief Gaussian noise generator
    private: double GaussianKernel(double mu, double sigma);

    /// \brief for setting ROS name space
    private: std::string robot_namespace_;

    private: ros::CallbackQueue queue_;
    private: void QueueThread();
    private: boost::thread callback_queue_thread_;

    // Pointer to the update event connection
    private: event::ConnectionPtr update_connection_;

    private: unsigned int seed;

    // ros publish multi queue, prevents publish() blocking
    private: PubMultiQueue pmq;

  };
}
#endif
