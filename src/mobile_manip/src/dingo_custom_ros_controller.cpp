#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include "realtime_tools/realtime_publisher.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/Marker.h>
#include "jackal_msgs/Drive.h"
#include "jackal_msgs/Feedback.h"
#include <diff_drive_controller/odometry.h>
#include <tf/tf.h>
#include <tf/tfMessage.h>

namespace dingo_custom_ros_controller{

    class DingoCustomRosController : public controller_interface::Controller<hardware_interface::VelocityJointInterface>
    {
    public:
        DingoCustomRosController()
            : base_frame_id_("base_link")
            , odom_frame_id_("odom")
            , enable_odom_tf_(true)
            , wheel_joints_size_(0)
            , cmd_vel_timeout_(0.5)
        {
        }

        bool init(hardware_interface::VelocityJointInterface* hw,
            ros::NodeHandle& root_nh,
            ros::NodeHandle &controller_nh)
        {
            const std::string complete_ns = controller_nh.getNamespace();
            std::size_t id = complete_ns.find_last_of("/");
            name_ = complete_ns.substr(id + 1);

            // Get joint names from the parameter server
            std::vector<std::string> left_wheel_names, right_wheel_names;
            if (!getWheelNames(controller_nh, "left_wheel", left_wheel_names) ||
                !getWheelNames(controller_nh, "right_wheel", right_wheel_names))
            {
                return false;
            }

            if (left_wheel_names.size() != right_wheel_names.size())
            {
                ROS_ERROR_STREAM_NAMED(name_,
                    "#left wheels (" << left_wheel_names.size() << ") != " <<
                    "#right wheels (" << right_wheel_names.size() << ").");
                return false;
            }
            else
            {
                wheel_joints_size_ = left_wheel_names.size();

                left_wheel_joints_.resize(wheel_joints_size_);
                right_wheel_joints_.resize(wheel_joints_size_);
            }

            // Get the joint object to use in the realtime loop
            for (size_t i = 0; i < wheel_joints_size_; ++i)
            {
                ROS_INFO_STREAM_NAMED(name_,
                                        "Adding left wheel with joint name: " << left_wheel_names[i]
                                        << " and right wheel with joint name: " << right_wheel_names[i]);
                left_wheel_joints_[i] = hw->getHandle(left_wheel_names[i]);  // throws on failure
                right_wheel_joints_[i] = hw->getHandle(right_wheel_names[i]);  // throws on failure
            }

            controller_nh.param("enable_odom_tf", enable_odom_tf_, enable_odom_tf_);

            // Odometry related
            double publish_rate = 50.0;
            publish_period_ = ros::Duration(1.0 / publish_rate);

            int velocity_rolling_window_size = 10;
            odometry_.setVelocityRollingWindowSize(velocity_rolling_window_size);
            odometry_.setWheelParams(ws_, lwr_, rwr_);

            setOdomPubFields(root_nh, controller_nh);
            vis_pub = controller_nh.advertise<visualization_msgs::Marker>( "dingo_vismarker", 0 );

            // Init publisher and subscriber
            cmd_drive_sub_ = controller_nh.subscribe("cmd_drive", 1, &DingoCustomRosController::cmdDriveCallback, this);
            sub_command_ = controller_nh.subscribe("cmd_vel", 1, &DingoCustomRosController::cmdVelCallback, this);

            // Realtime publisher, initializes differently from regular ros::Publisher
            setFeedbackPubFields(root_nh, controller_nh);
            //feedback_pub_.init(controller_nh, "feedback", 1);

            return true;
        }

        void update(const ros::Time& time, const ros::Duration& period)
        {
            double left_pos  = 0.0;
            double right_pos = 0.0;
            double left_vel = 0.0;
            double right_vel = 0.0;
            for (size_t i = 0; i < wheel_joints_size_; ++i)
            {
                const double lp = left_wheel_joints_[i].getPosition();
                const double rp = right_wheel_joints_[i].getPosition();
                const double lv = left_wheel_joints_[i].getVelocity();
                const double rv = right_wheel_joints_[i].getVelocity();
                if (std::isnan(lp) || std::isnan(rp) || std::isnan(lv) || std::isnan(rv))
                    return;

                left_pos  += lp;
                right_pos += rp;
                left_vel  += lv;
                right_vel += rv;
            }
            left_pos  /= wheel_joints_size_;
            right_pos /= wheel_joints_size_;
            left_vel  /= wheel_joints_size_;
            right_vel /= wheel_joints_size_;

            odometry_.update(left_pos, right_pos, time);

            publishFeedback(left_pos, left_vel, right_pos, right_vel);

            // Publish odometry message
            if (last_state_publish_time_ + publish_period_ < time)
            {
                last_state_publish_time_ += publish_period_;
                // Compute and store orientation info
                const geometry_msgs::Quaternion orientation(
                        tf::createQuaternionMsgFromYaw(odometry_.getHeading()));

                // Populate odom message and publish
                if (enable_odom_tf_ && odom_pub_->trylock())
                {
                    odom_pub_->msg_.header.stamp = time;
                    odom_pub_->msg_.pose.pose.position.x = odometry_.getX();
                    odom_pub_->msg_.pose.pose.position.y = odometry_.getY();
                    odom_pub_->msg_.pose.pose.orientation = orientation;
                    odom_pub_->msg_.twist.twist.linear.x  = odometry_.getLinear();
                    odom_pub_->msg_.twist.twist.angular.z = odometry_.getAngular();
                    odom_pub_->unlockAndPublish();
                }

		// Populate visualization_msg and publish
		visualization_msgs::Marker marker;
		marker.header.frame_id = "base_link";
		marker.header.stamp = ros::Time();
		marker.ns = "";
		marker.id = 0;
		marker.type = visualization_msgs::Marker::CUBE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = odometry_.getX();
		marker.pose.position.y = odometry_.getY();
		marker.pose.position.z = 0;
		marker.pose.orientation =  orientation;
		marker.scale.x = 0.2;
		marker.scale.y = 0.1;
		marker.scale.z = 0.1;
		marker.color.a = 1.0; // Don't forget to set the alpha!
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;
		vis_pub.publish( marker );

                // Publish tf /odom frame
                if (enable_odom_tf_ && tf_odom_pub_->trylock())
                {
                    geometry_msgs::TransformStamped& odom_frame = tf_odom_pub_->msg_.transforms[0];
                    odom_frame.header.stamp = time;
                    odom_frame.transform.translation.x = odometry_.getX();
                    odom_frame.transform.translation.y = odometry_.getY();
                    odom_frame.transform.rotation = orientation;
                    tf_odom_pub_->unlockAndPublish();
                }
            }


            // Time since last commands
            const double cmd_vel_dt   = (time - cmd_vel_stamp_).toSec();

            // Compute wheels velocities:
            double vel_left = 0.0;
            double vel_right = 0.0;

            if (cmd_vel_dt <= cmd_vel_timeout_)
            {
                vel_left  = (linear_ - angular_ * ws_ / 2.0)/lwr_;
                vel_right = (linear_ + angular_ * ws_ / 2.0)/rwr_;
            }
            else
            {
                    vel_left = left_wheel_command_;
                    vel_right = right_wheel_command_;
            }

            // Set wheels velocities:
            for (size_t i = 0; i < wheel_joints_size_; ++i)
            {
                left_wheel_joints_[i].setCommand(vel_left);
                right_wheel_joints_[i].setCommand(vel_right);
            }
        }

        void setOdomPubFields(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
        {
            // Setup odometry realtime publisher + odom message constant fields
            odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(controller_nh, "odom", 100));
            odom_pub_->msg_.header.frame_id = odom_frame_id_;
            odom_pub_->msg_.child_frame_id = base_frame_id_;
            odom_pub_->msg_.pose.pose.position.z = 0;
            odom_pub_->msg_.twist.twist.linear.y  = 0;
            odom_pub_->msg_.twist.twist.linear.z  = 0;
            odom_pub_->msg_.twist.twist.angular.x = 0;
            odom_pub_->msg_.twist.twist.angular.y = 0;
            tf_odom_pub_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(root_nh, "/tf", 100));
            tf_odom_pub_->msg_.transforms.resize(1);
            tf_odom_pub_->msg_.transforms[0].transform.translation.z = 0.0;
            tf_odom_pub_->msg_.transforms[0].child_frame_id = base_frame_id_;
            tf_odom_pub_->msg_.transforms[0].header.frame_id = odom_frame_id_;
        }

        void setFeedbackPubFields(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
        {
            // Setup Feedback realtime publisher + odom message constant fields
            feedback_pub_.reset(new realtime_tools::RealtimePublisher<jackal_msgs::Feedback>(controller_nh, "feedback", 1));
            feedback_pub_->msg_.header.frame_id = odom_frame_id_;
            feedback_pub_->msg_.drivers[jackal_msgs::Drive::LEFT].measured_travel = 0;
            feedback_pub_->msg_.drivers[jackal_msgs::Drive::LEFT].measured_velocity = 0;
            feedback_pub_->msg_.drivers[jackal_msgs::Drive::RIGHT].measured_travel = 0;
            feedback_pub_->msg_.drivers[jackal_msgs::Drive::RIGHT].measured_velocity = 0;
        }

        void publishFeedback(double left_travel, double left_velocity, double right_travel, double right_velocity)
        {
            if (feedback_pub_->trylock())
            {
                feedback_pub_->msg_.drivers[jackal_msgs::Drive::LEFT].measured_travel = left_travel;
                feedback_pub_->msg_.drivers[jackal_msgs::Drive::LEFT].measured_velocity = left_velocity;
                feedback_pub_->msg_.drivers[jackal_msgs::Drive::RIGHT].measured_travel = right_travel;
                feedback_pub_->msg_.drivers[jackal_msgs::Drive::RIGHT].measured_velocity = right_velocity;
                feedback_pub_->unlockAndPublish();
            }
        }

        void brake()
        {
            const double vel = 0.0;
            for (size_t i = 0; i < wheel_joints_size_; ++i)
            {
                left_wheel_joints_[i].setCommand(vel);
                right_wheel_joints_[i].setCommand(vel);
            }
        }

        void starting(const ros::Time& time)
        {
            brake();

            // Register starting time used to keep fixed rate
            last_state_publish_time_ = time;

            odometry_.init(time);
        }

        void stopping(const ros::Time& time) { }

    private:
        std::string name_;

        /// Hardware handles:
        std::vector<hardware_interface::JointHandle> left_wheel_joints_;
        std::vector<hardware_interface::JointHandle> right_wheel_joints_;

        /// Number of wheel joints:
        size_t wheel_joints_size_;

        /// Controller parameters
        const double ws_  = 0.432;
        const double lwr_ = 0.049;
        const double rwr_ = 0.049;

        /// Timeout to consider cmd_vel commands old
        double cmd_vel_timeout_;

        /// Timestamp of the most recent cmd_vel command
        ros::Time cmd_vel_stamp_ = ros::Time(0);

        // Velocity setpoints
        double linear_ = 0.0;
        double angular_ = 0.0;

        // Wheel command setpoints
        double left_wheel_command_ = 0.0;
        double right_wheel_command_ = 0.0;

        // Frame to use for the robot base
        std::string base_frame_id_;

        // Frame to use for odometry and odom tf
        std::string odom_frame_id_;

        // Whether to publish odometry to tf or not
        bool enable_odom_tf_;

        // Publisher
        std::shared_ptr<realtime_tools::RealtimePublisher<jackal_msgs::Feedback> > feedback_pub_;
        ros::Publisher vis_pub;

        // Subscriber
        ros::Subscriber cmd_drive_sub_;
        ros::Subscriber sub_command_;

        // Odometry related
        ros::Duration publish_period_;
        ros::Time last_state_publish_time_;
        std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> > odom_pub_;
        std::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage> > tf_odom_pub_;
        diff_drive_controller::Odometry odometry_;

        void cmdDriveCallback(const jackal_msgs::Drive::ConstPtr& msg)
        {
            if (isRunning())
            {
                left_wheel_command_  = msg->drivers[jackal_msgs::Drive::LEFT];
                right_wheel_command_ = msg->drivers[jackal_msgs::Drive::RIGHT];
                // ROS_INFO_STREAM("Received new commands. "
                //     << "Left: "   << left_wheel_command_ << ", "
                //     << "Right: "   << right_wheel_command_);
            }
            else
            {
                ROS_INFO_STREAM("Can't accept new commands. Controller is not running.");
            }
        }

        void cmdVelCallback(const geometry_msgs::Twist& command)
        {
            if (isRunning())
            {
                cmd_vel_stamp_ = ros::Time::now();
                angular_  = command.angular.z;
                linear_   = command.linear.x;
            }
            else
            {
                ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
            }
        }

        bool getWheelNames(ros::NodeHandle& controller_nh,
            const std::string& wheel_param,
            std::vector<std::string>& wheel_names)
        {
            XmlRpc::XmlRpcValue wheel_list;
            if (!controller_nh.getParam(wheel_param, wheel_list))
            {
                ROS_ERROR_STREAM_NAMED(name_,
                    "Couldn't retrieve wheel param '" << wheel_param << "'.");
                return false;
            }

            if (wheel_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
            {
                if (wheel_list.size() == 0)
                {
                    ROS_ERROR_STREAM_NAMED(name_,
                        "Wheel param '" << wheel_param << "' is an empty list");
                    return false;
                }

                for (int i = 0; i < wheel_list.size(); ++i)
                {
                    if (wheel_list[i].getType() != XmlRpc::XmlRpcValue::TypeString)
                    {
                        ROS_ERROR_STREAM_NAMED(name_,
                            "Wheel param '" << wheel_param << "' #" << i <<
                            " isn't a string.");
                        return false;
                    }
                }

                wheel_names.resize(wheel_list.size());
                for (int i = 0; i < wheel_list.size(); ++i)
                {
                    wheel_names[i] = static_cast<std::string>(wheel_list[i]);
                }
            }
            else if (wheel_list.getType() == XmlRpc::XmlRpcValue::TypeString)
            {
                wheel_names.push_back(wheel_list);
            }
            else
            {
                ROS_ERROR_STREAM_NAMED(name_,
                    "Wheel param '" << wheel_param <<
                    "' is neither a list of strings nor a string.");
                return false;
            }

            return true;
        }
    };
    PLUGINLIB_EXPORT_CLASS(dingo_custom_ros_controller::DingoCustomRosController, controller_interface::ControllerBase);
    // PLUGINLIB_DECLARE_CLASS(package_name, DingoCustomRosController, dingo_custom_ros_controller::DingoCustomRosController, controller_interface::ControllerBase);
}//namespace
