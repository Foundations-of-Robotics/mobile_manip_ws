from filterpy.kalman import ExtendedKalmanFilter
from math import pi, sin, cos
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

class JackalCustomEKF(ExtendedKalmanFilter):
    def __init__(self, dim_x, dim_z):
        super(JackalCustomEKF, self).__init__(dim_x=dim_x, dim_z=dim_z)

        self.dt = 0.1
        self.last_measurement_time = 0
        self.set_qf()

    def set_qf(self):
        dt = self.dt
        yaw = self.get_yaw()

        # As we integrate to find position, we lose precision. Thus we trust x less than dx/dt, hence the dt*2 vs dt.
        self.Q = np.array([[dt*0.05, 0,        0,       0,       0,        0,       0,       0      ],
                           [0,       dt*0.025, 0,       0,       0,        0,       0,       0      ],
                           [0,       0,        dt*0.01, 0,       0,        0,       0,       0      ],
                           [0,       0,        0,       dt*0.05, 0,        0,       0,       0      ],
                           [0,       0,        0,       0,       dt*0.025, 0,       0,       0      ],
                           [0,       0,        0,       0,       0,        dt*0.01, 0,       0      ],
                           [0,       0,        0,       0,       0,        0,       dt*0.06, 0      ],
                           [0,       0,        0,       0,       0,        0,       0,       dt*0.02]])

        self.F = np.array([[1, cos(yaw)*dt, cos(yaw)*dt*dt/2, 0, -sin(yaw)*dt,  -sin(yaw)*dt*dt/2, 0, 0 ],
                           [0, 1,           dt,               0, 0,             0,                 0, 0 ],
                           [0, 0,           1,                0, 0,             0,                 0, 0 ],
                           [0, sin(yaw)*dt, sin(yaw)*dt*dt/2, 1, cos(yaw)*dt,   cos(yaw)*dt*dt/2,  0, 0 ],
                           [0, 0,           0,                0, 1,             dt,                0, 0 ],
                           [0, 0,           0,                0, 0,             1,                 0, 0 ],
                           [0, 0,           0,                0, 0,             0,                 1, dt],
                           [0, 0,           0,                0, 0,             0,                 0, 1 ]])

    def get_x(self):
        return self.x[0]

    def get_y(self):
        return self.x[3]

    def get_yaw(self):
        return self.x[6]

    def get_vx(self):
        return self.x[1]

    def get_vyaw(self):
        return self.x[7]

    def Hx_odometry(self, x):
        return np.dot(self.H_odometry, x)

    def Hx_imu(self, x):
        return np.dot(self.H_imu, x)

    def pre_update(self, timestamp):
        if timestamp > self.last_measurement_time:
            self.dt = timestamp - self.last_measurement_time
            self.last_measurement_time = timestamp
            self.set_qf()
        else:
            pass
            # print("Received message with bad timestamp.")
        self.predict()

    def odometry_update(self, z, timestamp):
        self.pre_update(timestamp)
        super(JackalCustomEKF, self).update(z, lambda _: self.H_odometry, self.Hx_odometry, self.R_odometry)

    def imu_update(self, z, timestamp):
        self.pre_update(timestamp)
        super(JackalCustomEKF, self).update(z, lambda _: self.H_imu, self.Hx_imu, self.R_imu)
 
   # Publishes odometry msgs and tf transform
    def publish_odom_tf(self, odom_pub):
       time = rospy.Time.now()
       odom_frame = "odom"
       base_frame = "base_link"
       # Get state from this file pose = (x_, y_, 0.) orientation = tf.transformations.quaternion_from_euler(0, 0, 
       # heading_) linear = linear_vel_ angular = angular_vel_ Get state from ekf
       pose = (self.get_x(), self.get_y(), 0.)
       r = R.from_euler('zyx', [0., 0., self.get_yaw()], degrees=False)
       orientation = r.as_quat()
       vlin = self.get_vx()
       vang = self.get_vyaw()
       # Publish odometry message
       odom_msg = Odometry()
       odom_msg.header.frame_id = odom_frame
       odom_msg.header.stamp = time
       odom_msg.child_frame_id = base_frame
       odom_msg.pose.pose = Pose(Point(*pose), Quaternion(*orientation))
       odom_msg.twist.twist = Twist(Vector3(vlin, 0, 0), Vector3(0, 0, vang))
       odom_pub.publish(odom_msg)
       # Broadcast tf transform
#       br = TransformBroadcaster()
#       br.sendTransform(pose, orientation, time, base_frame, odom_frame)
