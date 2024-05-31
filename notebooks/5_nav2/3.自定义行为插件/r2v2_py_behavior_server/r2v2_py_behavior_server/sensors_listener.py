# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from time import sleep
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup,MutuallyExclusiveCallbackGroup

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D,PoseWithCovarianceStamped,Twist,Point
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.executors import MultiThreadedExecutor

from queue import Queue
from scipy.spatial.transform import Rotation
class Listener(Node):

    def __init__(self):
        super().__init__('sensors_listener')
        # self.imu_listener_init()
        # self.ops_listener_init()
        self.cmd_vel_listener_init()
        lio_group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(
            Odometry,
            '/Odometry',
            self.fastlio_odom_callback,
            10,
            callback_group=lio_group )
        self.filtered_odom_publisher = self.create_publisher(
            Odometry,
            'filtered_odom',
            3
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.04,self.timer_callback,callback_group=lio_group)
        
        self.first_sub_time = 0
        self.init_lio_msg=Queue(maxsize=10)
        self.lio_msg = Odometry()
        self.lio_offset =Odometry()
        self.state = 'init'

        # self.calculate_odom_offset()

    def fastlio_odom_callback(self,msg):
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        dt = current_time-self.first_sub_time
        if self.state == 'init':
            self.first_sub_time = current_time
            self.state='waiting'
            self.get_logger().info(f'Current state: {self.state}')
        elif self.state == 'waiting':
            if dt>3:
                self.state = 'recording'
                self.get_logger().info(f'Current state: {self.state}')
        elif self.state == 'recording':
            if self.init_lio_msg.full():
                self.init_lio_msg.get()
            self.init_lio_msg.put(msg)
            if dt>8 and self.init_lio_msg.full():
                self.state = 'init done'
        else:
            self.lio_msg = msg


    def calculate_odom_offset(self):
        init_lio_msg_list = list(self.init_lio_msg.queue)
        positions=[x.pose.pose.position for x in init_lio_msg_list]
        orientations=[x.pose.pose.orientation for x in init_lio_msg_list]
        lenght = len(init_lio_msg_list)
        avg_pos = Point()
        orientations_list =[]
        for p,o in zip(positions,orientations):
            avg_pos.x+=p.x
            avg_pos.y+=p.y
            avg_pos.z+=p.z
            orientations_list.append([o.x,o.y,o.z,o.w])
        avg_pos.x/=lenght
        avg_pos.y/=lenght
        avg_pos.z/=lenght
        avg_orientation = Rotation.from_quat(orientations_list).mean().as_quat()
        self.lio_offset.pose.pose.position = avg_pos
        self.lio_offset.pose.pose.orientation.x =avg_orientation[0]
        self.lio_offset.pose.pose.orientation.y =avg_orientation[1]
        self.lio_offset.pose.pose.orientation.z =avg_orientation[2]
        self.lio_offset.pose.pose.orientation.w =avg_orientation[3]



    def timer_callback(self):
        if self.state == 'init done':
            self.get_logger().info(f'Current state: {self.state}')
            self.calculate_odom_offset()
            self.state = 'running'
        elif self.state != 'running':
            # self.get_logger().info(f'Current state: {self.state}')
            return
        T = TransformStamped()
        odom_msg = Odometry()
#         child_frame_id：这表示机器人的某个部分（比如底盘）相对于 header.frame_id 所在的坐标系的位置和速度。
#         header.frame_id：这表示 child_frame_id 所在的坐标系。
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.pose.pose.position.x = self.lio_msg.pose.pose.position.x-self.lio_offset.pose.pose.position.x
        odom_msg.pose.pose.position.y = self.lio_msg.pose.pose.position.y-self.lio_offset.pose.pose.position.y
        odom_msg.pose.pose.position.z = self.lio_msg.pose.pose.position.z-self.lio_offset.pose.pose.position.z
        q1 = Rotation.from_quat([self.lio_msg.pose.pose.orientation.x,self.lio_msg.pose.pose.orientation.y,self.lio_msg.pose.pose.orientation.z,self.lio_msg.pose.pose.orientation.w])
        q2 = Rotation.from_quat([self.lio_offset.pose.pose.orientation.x,self.lio_offset.pose.pose.orientation.y,self.lio_offset.pose.pose.orientation.z,self.lio_offset.pose.pose.orientation.w])
        q = (q1*q2.inv()).as_quat()
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        odom_msg.pose.covariance=self.lio_msg.pose.covariance
        odom_msg.twist.twist.linear = self.lio_msg.twist.twist.linear
        odom_msg.twist.twist.angular = self.lio_msg.twist.twist.angular
        odom_msg.twist.covariance= self.lio_msg.twist.covariance 
        

        T.header.stamp = odom_msg.header.stamp
        T.header.frame_id = 'odom'
        T.child_frame_id = 'base_link'
        T.transform.translation.x = odom_msg.pose.pose.position.x
        T.transform.translation.y = odom_msg.pose.pose.position.y
        T.transform.translation.z = odom_msg.pose.pose.position.z
        T.transform.rotation.x = odom_msg.pose.pose.orientation.x
        T.transform.rotation.y = odom_msg.pose.pose.orientation.y
        T.transform.rotation.z = odom_msg.pose.pose.orientation.z
        T.transform.rotation.w = odom_msg.pose.pose.orientation.w

        # self.get_logger().info('Publishing filtered odom')
        self.tf_broadcaster.sendTransform(T)
        self.filtered_odom_publisher.publish(odom_msg)

    # def imu_listener_init(self):
        
    #     self.create_subscription(
    #         Float32MultiArray,
    #         'car/imu',
    #         self.imu_callback,
    #         3,
    #         callback_group=ReentrantCallbackGroup() )
    #     self.imu_publisher = self.create_publisher(Imu, 'demo/imu', 5)
        
    # def imu_callback(self, msg):
    #     imu_msg = Imu()
    #     imu_msg.header.frame_id = 'base_link'
    #     #header.frame_id 的作用是提供关于IMU传感器测量的物理意义的信息，即它说明了测量数据是在哪个坐标系下进行的。
    #     imu_msg.header.stamp = self.get_clock().now().to_msg()
    #     R=Rotation.from_euler('z', msg.data[2], degrees=False)
    #     self.q = R.as_quat()
    #     imu_msg.orientation.x = self.q[0]
    #     imu_msg.orientation.y = self.q[1]
    #     imu_msg.orientation.z = self.q[2]
    #     imu_msg.orientation.w = self.q[3]
    #     imu_msg.orientation_covariance = [  0.01, 0.0, 0.0,
    #                                         0.0, 0.01, 0.0, 
    #                                         0.0, 0.0, 0.01]
    #     imu_msg.angular_velocity.x = 0.0
    #     imu_msg.angular_velocity.y =msg.data[0]
    #     imu_msg.angular_velocity.z = 0.0
    #     imu_msg.angular_velocity_covariance = [0.01, 0.0, 0.0, 
    #                                            0.0, 0.01, 0.0, 
    #                                            0.0, 0.0, 0.01]
    #     imu_msg.linear_acceleration.x = 0.0
    #     imu_msg.linear_acceleration.y = 0.0
    #     imu_msg.linear_acceleration.z = 0.0
    #     imu_msg.linear_acceleration_covariance = [0.1, 0.0, 0.0, 
    #                                               0.0, 0.1, 0.0, 
    #                                               0.0, 0.0, 0.1]

    #     self.imu_publisher.publish(imu_msg)
    
    # def ops_listener_init(self):
    #     self.create_subscription(
    #         Pose2D,
    #         'car/ops',
    #         self.ops_callback,
    #         3,
    #         callback_group=ReentrantCallbackGroup() )
    #     self.odom_publisher = self.create_publisher(Odometry, 'demo/odom', 5)
#     def ops_callback(self,msg):
#         #_____
#         self.pose_2d =[msg.x,msg.y]
#         #_____
#         odom_msg = Odometry()
# #         child_frame_id：这表示机器人的某个部分（比如底盘）相对于 header.frame_id 所在的坐标系的位置和速度。
# #         header.frame_id：这表示 child_frame_id 所在的坐标系。
#         odom_msg.header.frame_id = 'odom'
#         odom_msg.child_frame_id = 'base_link'
#         odom_msg.header.stamp = self.get_clock().now().to_msg()
#         odom_msg.pose.pose.position.x = msg.x
#         odom_msg.pose.pose.position.y = msg.y
#         odom_msg.pose.pose.position.z = 0.0
#         odom_msg.pose.pose.orientation.x = 0.0
#         odom_msg.pose.pose.orientation.y = 0.0
#         odom_msg.pose.pose.orientation.z = 0.0
#         odom_msg.pose.pose.orientation.w = 1.0
#         odom_msg.pose.covariance=[  0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
#                                     0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
#                                     0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
#                                     0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
#                                     0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
#                                     0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
#         odom_msg.twist.twist.linear.x = 0.0
#         odom_msg.twist.twist.linear.y = 0.0
#         odom_msg.twist.twist.linear.z = 0.0
#         odom_msg.twist.twist.angular.x = 0.0
#         odom_msg.twist.twist.angular.y = 0.0
#         odom_msg.twist.twist.angular.z = 0.0
#         odom_msg.twist.covariance=[ 0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
#                                     0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
#                                     0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
#                                     0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
#                                     0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
#                                     0.0, 0.0, 0.0, 0.0, 0.0, 0.01]
#         self.odom_publisher.publish(odom_msg)
    
    def cmd_vel_listener_init(self):
        self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            3,
            callback_group=ReentrantCallbackGroup() )
        self.cmd_vel_publisher = self.create_publisher(Pose2D, 'car/cmd_vel', 3)

    def cmd_vel_callback(self,msg):
        cmd_vel_msg = Pose2D()
        cmd_vel_msg.x = msg.linear.x
        cmd_vel_msg.y = msg.linear.y
        cmd_vel_msg.theta = msg.angular.z
        self.cmd_vel_publisher.publish(cmd_vel_msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = Listener()    
    executor = MultiThreadedExecutor()
    executor.add_node(minimal_publisher)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        rclpy.shutdown()



if __name__ == '__main__':
    main()
