#!/usr/bin/env python

# Simple node that subscribes to the Imu and wheel encoders and estimate Dingo's pose

import rospy
import tf
import numpy as np
from math import sin, cos
from jackal_msgs.msg import Feedback
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from jackal_custom_filters.ekf import JackalCustomEKF


# Variables to store data from subscribed topics
left_pos = 0.0
right_pos = 0.0
imu_msg = Imu()

# Variables to store current odometry
x_ = 0.0
y_ = 0.0
heading_ = 0.0
linear_vel_ = 0.0
angular_vel_ = 0.0

# Robot wheels parameters in [m]
wheel_separation = 0.3765
left_wheel_radius = 0.098
right_wheel_radius = 0.098

# Create the Kalman Filter object
ekf = JackalCustomEKF(dim_x=8, dim_z=8)
use_acc_on_kalman_filter = False


# Imu subscriber callback
def imu_callback(msg):
    imu_msg.orientation.x = msg.orientation.x
    imu_msg.orientation.y = msg.orientation.y
    imu_msg.orientation.z = msg.orientation.z
    imu_msg.orientation.w = msg.orientation.w
    imu_msg.angular_velocity.x = msg.angular_velocity.x
    imu_msg.angular_velocity.y = msg.angular_velocity.y
    imu_msg.angular_velocity.z = msg.angular_velocity.z
    imu_msg.linear_acceleration.x = msg.linear_acceleration.x
    imu_msg.linear_acceleration.y = msg.linear_acceleration.y
    imu_msg.linear_acceleration.z = msg.linear_acceleration.z


    if use_acc_on_kalman_filter:
        ekf.imu_update(np.array([
            get_heading_from_imu(), 
            imu_msg.angular_velocity.z, 
            imu_msg.linear_acceleration.x,
            imu_msg.linear_acceleration.y]),
            rospy.get_time()
        )
    else:
        ekf.imu_update(np.array([
            get_heading_from_imu(), 
            imu_msg.angular_velocity.z]),
            rospy.get_time()
        )


# Calculate heading from imu_msg
def get_heading_from_imu():
    angles = tf.transformations.euler_from_quaternion([
        imu_msg.orientation.x, 
        imu_msg.orientation.y, 
        imu_msg.orientation.z, 
        imu_msg.orientation.w
    ])
    return angles[2]


# Encoders subscriber callback
def feedback_callback(msg):
    global left_pos
    global right_pos
    left_pos  = msg.drivers[0].measured_travel
    right_pos = msg.drivers[1].measured_travel


# Publishes odometry msgs and tf transform
def publish_odom_tf():
    time = rospy.Time.now()
    odom_frame = "odom"
    base_frame = "base_link"

    # Get state from this file
    # pose = (x_, y_, 0.)
    # orientation = tf.transformations.quaternion_from_euler(0, 0, heading_)
    # linear = linear_vel_
    # angular = angular_vel_

    # Get state from ekf
    pose = (ekf.get_x(), ekf.get_y(), 0.)
    orientation = tf.transformations.quaternion_from_euler(0, 0, ekf.get_yaw())
    linear = ekf.get_vx()
    angular = ekf.get_vyaw()

    # Publish odometry message
    odom_msg = Odometry()
    odom_msg.header.frame_id = odom_frame
    odom_msg.header.stamp = time
    odom_msg.child_frame_id = base_frame
    odom_msg.pose.pose = Pose(Point(*pose), Quaternion(*orientation))
    odom_msg.twist.twist = Twist(Vector3(linear_vel_, 0, 0), Vector3(0, 0, angular_vel_))
    odom_pub.publish(odom_msg)

    # Broadcast tf transform
    br = tf.TransformBroadcaster()
    br.sendTransform(pose, orientation, time, base_frame, odom_frame)


# Computes odometry from encoders data
prev_left_pos = 0.0
prev_right_pos = 0.0
def compute_odometry(left_pos, right_pos):

    global prev_left_pos, prev_right_pos, x_, y_, heading_, linear_vel_, angular_vel_, last_time_

    # Here you have to use the wheel parameters and the data from the encoders
    # to compute the robot's current pose and store in the variables x, y and heading

    left_wheel_cur_pos  = left_pos * left_wheel_radius
    right_wheel_cur_pos = right_pos * right_wheel_radius

    left_wheel_est_vel  = left_wheel_cur_pos - prev_left_pos
    right_wheel_est_vel = right_wheel_cur_pos - prev_right_pos

    prev_left_pos  = left_wheel_cur_pos
    prev_right_pos = right_wheel_cur_pos

    linear  = (right_wheel_est_vel + left_wheel_est_vel) * 0.5
    angular = (right_wheel_est_vel - left_wheel_est_vel) / wheel_separation

    if abs(angular) < 1e-6:
        direction = heading_ + angular * 0.5

        x_       += linear * cos(direction)
        y_       += linear * sin(direction)
        heading_ += angular
    else:
        heading_old = heading_
        r = linear/angular
        heading_ += angular
        x_       +=  r * (sin(heading_) - sin(heading_old))
        y_       += -r * (cos(heading_) - cos(heading_old))

    time = rospy.Time.now()
    dt = (time - last_time_).to_sec()
    if dt < 0.0001:
        return

    last_time_ = time

    linear_vel_  = linear/dt
    angular_vel_ = angular/dt

    ekf.odometry_update(np.array([linear_vel_, angular_vel_]), rospy.get_time())


# Here you can set the Kalman Filter parameters
def configure_kalman_filter():
    dt = 1/50

    # Initial state
    ekf.x = np.zeros(8)

    # Process uncertainty matrix
    # ekf.Q = np.array([[dt*0.05, 0,    0,  0,    0,    0,  0,    0],
    #                   [0,    dt*0.025, 0,  0,    0,    0,  0,    0],
    #                   [0,    0,    dt*0.01, 0,    0,    0,  0,    0],
    #                   [0,    0,    0,  dt*0.05, 0,    0,  0,    0],
    #                   [0,    0,    0,  0,    dt*0.025, 0,  0,    0],
    #                   [0,    0,    0,  0,    0,    dt*0.01, 0,    0],
    #                   [0,    0,    0,  0,    0,    0,  dt*0.06, 0],
    #                   [0,    0,    0,  0,    0,    0,  0,    dt*0.02]])

    # # State transition matrix
    # ekf.F = np.array([[1, dt, dt*dt/2, 0, 0,  0,       0, 0 ],
    #                   [0, 1,  dt,      0, 0,  0,       0, 0 ],
    #                   [0, 0,  0,       0, 0,  0,       0, 0 ],
    #                   [0, 0,  0,       1, dt, dt*dt/2, 0, 0 ],
    #                   [0, 0,  0,       0, 1,  dt,      0, 0 ],
    #                   [0, 0,  0,       0, 0,  1,       0, 0 ],
    #                   [0, 0,  0,       0, 0,  0,       1, dt],
    #                   [0, 0,  0,       0, 0,  0,       0, 1 ]])

    # Odometry measurement noise matrix
    ekf.R_odometry = np.array([
        [0.001, 0   ],
        [0,     0.03]
    ])

    # Odometry measurement function
    ekf.H_odometry = np.array([
        [0, 1, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 1]
    ])

    # IMU measurement noise matrix
    if use_acc_on_kalman_filter:
        ekf.R_imu = np.array([
            [0.00000026, 0,        0,        0       ],
            [0,          0.000025, 0,        0       ],
            [0,          0,        0.000025, 0       ],
            [0,          0,        0,        0.000025]
        ])
    else:
        ekf.R_imu = np.array([
            [0.00000026, 0,      ],
            [0,          0.000025]
        ])

    # IMU measurement function
    if use_acc_on_kalman_filter:
        ekf.H_imu =  np.array([
            [0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 1],
            [0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 1, 0, 0]
        ])
    else:
        ekf.H_imu = np.array([
            [0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 1]
        ])


if __name__ == '__main__':
    try:
        # Starts a new node
        rospy.init_node('dingo_localization', anonymous=True)
        rate = rospy.Rate(50) # 50hz
        global last_time_ 
        last_time_ = rospy.Time.now()

        # ROS subscribers and publishers
        odom_pub = rospy.Publisher('/dingo_velocity_controller/odom', Odometry, queue_size=1)
        feedback_sub = rospy.Subscriber('/dingo_velocity_controller/feedback', Feedback, feedback_callback)
        imu_sub = rospy.Subscriber('/imu/data', Imu, imu_callback)

        # Initialize Kalman Filter parameters
        configure_kalman_filter()

        while not rospy.is_shutdown():
            # Estimate current odometry from encoders data
            compute_odometry(left_pos, right_pos)

            # Publish odometry and tf transform
            publish_odom_tf()

            # Sleep until next loop
            rate.sleep()

    except rospy.ROSInterruptException: pass