"""底盘移动动作server"""
from typing import Iterator, List
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
import rclpy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose2D
from builtin_interfaces.msg import Time
import time
from rclpy.clock import Clock
from threading import Lock
from rc2024_interfaces.action import ChassisMoveTo
import numpy as np
import math
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
import threading
from ament_index_python.packages import get_package_share_directory
import os
# 获取包的共享目录
package_share_directory = get_package_share_directory('RC24_R2_v1_ros2')
# 构建.npy文件的完整路径
npy_file_path = os.path.join(package_share_directory, 'data', 'planner.npy')
# 加载.npy文件
planner_arr = np.load(npy_file_path)

Kx,Ky=2.0,1.0
Kptheta = 2.0
output_theta_max = 1.0
#开始减速的点(倒数第几个点)
point_start_decelerate = 50
class Chassis_node(Node):
    def __init__(self,name:str):
        super().__init__(name)
        self.pub = self.create_publisher(Pose2D,"car/cmd_vel",2)
        sub_cb_group = MutuallyExclusiveCallbackGroup()
        act_cb_group = MutuallyExclusiveCallbackGroup()
        self.sub = self.create_subscription(Pose2D,
                                            "car/ops",
                                            self.get_pose_speed_callback,
                                            2,callback_group=sub_cb_group)
        self.chassis_moveto_server = ActionServer(
            self, ChassisMoveTo, 'chassis_move_to', self.chassis_moveto_execute
            ,callback_group=act_cb_group
        )
        self.Mutex = Lock()    #变量锁
        #纯跟踪控制参数
        self.Kv=0.0
        self.ld0 = 0.1

        self.Pa = Pose2D() #actual_pose
        self.Pa.x = 0.
        self.Pa.y = 0.
        self.Pa.theta = 0.
        self.Pa_time_s = 0.
        self.Pa_time_ns = 0.
        
        self.Pa_last = Pose2D()
        self.Pa_last.x = 0.
        self.Pa_last.y = 0.
        self.Pa_last.theta = 0.
        self.Pa_last_time_s = 0.
        self.Pa_last_time_ns = 0.
        #self.planner = planner(num)
        
        self.Pv = Pose2D() #actual_velocity
        self.Pv.x = 0.
        self.Pv.y = 0.
        self.Pv.theta = 0.
    
    def get_pose_speed_callback(self,msgin):
        '''通过订阅底盘实际位置信息，计算底盘实际速度'''
        self.Pa_last.x = self.Pa.x
        self.Pa_last.y = self.Pa.y
        self.Pa_last.theta = self.Pa.theta
        self.Pa_last_time_s = self.Pa_time_s
        self.Pa_last_time_ns = self.Pa_time_ns
        
        with self.Mutex:
            self.Pa.x = msgin.x
            self.Pa.y = msgin.y
            self.Pa.theta = msgin.theta
            self.Pa_time_s,self.Pa_time_ns = self.get_clock().now().seconds_nanoseconds()
            
            delt_t = (self.Pa_time_s + self.Pa_time_ns*10**(-9)-self.Pa_last_time_s-self.Pa_last_time_ns*10**(-9))
            self.Pv.x = (self.Pa.x-self.Pa_last.x)/delt_t
            self.Pv.y = (self.Pa.y-self.Pa_last.y)/delt_t
            self.Pv.theta = (self.Pa.theta-self.Pa_last.theta)/delt_t
            #self.get_logger().info(f'actual pos {self.Pa.x:.3f}, {self.Pa.y:.3f},{self.Pa.theta:.3f}')

            #self.get_logger().info(f'delta {delt_t:f},{self.Pa_last_time_ns*10**(-9):f},{self.Pa_time_ns*10**(-9):f}')
    def v_planner(self,total_point,i_now):
            '''速度规划'''
            global planner_arr,Kx,Ky,point_start_decelerate

            if total_point<point_start_decelerate:
                Kp = planner_arr[round(i_now/total_point*planner_arr.shape[0])]
            else:
                if (p:=i_now-total_point+point_start_decelerate)>0:
                    Kp = planner_arr[round(p/point_start_decelerate*planner_arr.shape[0])]
                else:
                    Kp=1.0
            return Kx*Kp,Ky*Kp

    def find_Pld_index(self,Prs_list:Iterator,start_index=0):
        '''找到预瞄点的索引'''
        v = math.sqrt(self.Pv.x**2+self.Pv.y**2)
        ld = self.Kv*v+self.ld0
        i_min = 0
        l2_min = 100000
        for i,P in enumerate(Prs_list[start_index:]):
            i = i +start_index
            l2=(P[0]-self.Pa.x)**2+(P[1]-self.Pa.y)**2
            if  l2< l2_min:
                l2_min = l2
                i_min = i
            #self.get_logger().info(f'P:{Prs_list[i][0]:.2f},l2:{l2:.5f},l2_min:{l2_min:.5f},i_min:{i_min:d}')
        #self.get_logger().info(f'l2_min:{l2_min:.2f},i_min:{i_min:d}')
        l_min = abs(math.sqrt(l2_min)-ld)
        Pld_index = i_min
        for i,P in enumerate(Prs_list[i_min:]):
            l=abs(math.sqrt((P[0]-self.Pa.x)**2+(P[1]-self.Pa.y)**2)-ld)
            if l<l_min:
                l_min = l
                Pld_index = i_min+i
        #self.get_logger().info(f'v:{v:.2f},Pld:{Pld_index:d},i_min:{i_min:d}')

        return Pld_index,i_min


    # def mv_to_Pld(self,Pr,total_point,i_now):
    #     '''跟踪预瞄点'''
    #     global Kptheta ,output_theta_max
    #     Kpx ,Kpy= self.v_planner(total_point,i_now)
    #     Pe = Pose2D()
    #     with self.Mutex:
    #         Pe.x = (Pr.x-self.Pa.x)*math.cos(self.Pa.theta)+(Pr.y-self.Pa.y)*math.sin(self.Pa.theta)
    #         Pe.y = (Pr.y-self.Pa.y)*math.cos(self.Pa.theta)-(Pr.x-self.Pa.x)*math.sin(self.Pa.theta)
    #         Pe.theta = Pr.theta-self.Pa.theta
            
    #     Data_To_Pub = Pose2D()
    #     Data_To_Pub.x = Kpx*Pe.x
    #     Data_To_Pub.y = Kpy*Pe.y
    #     Data_To_Pub.theta = Kptheta*Pe.theta
    #     if(Data_To_Pub.theta>output_theta_max):
    #         Data_To_Pub.theta=output_theta_max
    #     elif(Data_To_Pub.theta<-output_theta_max):
    #         Data_To_Pub.theta=-output_theta_max
    #     self.pub.publish(Data_To_Pub)
    #     if Pe.x<0.01 and Pe.y<0.01 :
    #         i_now=i_now+1
    #     self.get_logger().info(f',Kpx:{Kpx},Pub x:{Data_To_Pub.x:.2f},w:{Data_To_Pub.theta:.2f}')


    def chassis_moveto_execute(self, goal_handle: ServerGoalHandle):
        '''底盘移动动作server'''
        feedback_msg = ChassisMoveTo.Feedback()
        dt = 0.05 #50ms
        total_time = 0.0
        Pld_index = 0
        progress = 0
        Prs_start = 0
        if goal_handle.request.xy_relative == True:
            goal_handle.request.xr = [x+self.Pa.x for x in goal_handle.request.xr]
            goal_handle.request.yr = [y+self.Pa.y for y in goal_handle.request.yr]
        if goal_handle.request.w_relative == True:
            goal_handle.request.wr = [w+self.Pa.theta for w in goal_handle.request.wr]
        Prs_list = list(zip(goal_handle.request.xr,goal_handle.request.yr,goal_handle.request.wr))
        Prs_len = len(Prs_list)

        while Pld_index<(Prs_len-1) and rclpy.ok():
            Pld_index,Prs_start = self.find_Pld_index(Prs_list,Prs_start)
            #Pr:此次目标点
            Pr = Pose2D()
            Pr.x = Prs_list[Pld_index][0]
            Pr.y = Prs_list[Pld_index][1]
            Pr.theta = Prs_list[Pld_index][2]
            #self.mv_to_Pld(Pr,Prs_len,Prs_start)

            global Kptheta ,output_theta_max
            Kpx ,Kpy= self.v_planner(Prs_len,Prs_start)
            Pe = Pose2D()
            with self.Mutex:
                Pe.x = (Pr.x-self.Pa.x)*math.cos(self.Pa.theta)+(Pr.y-self.Pa.y)*math.sin(self.Pa.theta)
                Pe.y = (Pr.y-self.Pa.y)*math.cos(self.Pa.theta)-(Pr.x-self.Pa.x)*math.sin(self.Pa.theta)
                Pe.theta = Pr.theta-self.Pa.theta
                if Pe.theta>math.pi:
                    Pe.theta-=2*math.pi
                elif Pe.theta<-math.pi:
                    Pe.theta+=2*math.pi
                
            Data_To_Pub = Pose2D()
            Data_To_Pub.x = Kpx*Pe.x
            Data_To_Pub.y = Kpy*Pe.y
            Data_To_Pub.theta = Kptheta*Pe.theta
            if(Data_To_Pub.theta>output_theta_max):
                Data_To_Pub.theta=output_theta_max
            elif(Data_To_Pub.theta<-output_theta_max):
                Data_To_Pub.theta=-output_theta_max
            self.pub.publish(Data_To_Pub)
            if abs(Pe.x)<0.05 and abs(Pe.y)<0.05 :
                if abs(Pe.theta)>0.1:
                    pass
                else:
                    Prs_start=Prs_start+1
            if all(x==0.0 for x in goal_handle.request.xr+goal_handle.request.yr):
                pass
            self.get_logger().info(f'Pe theta:{Pe.theta:.2f} target theta:{Pr.theta:.2f}')

            progress = float(Prs_start/Prs_len)


            self.get_logger().info(f'i               {progress:.2f}')
            feedback_msg.progress = progress
            goal_handle.publish_feedback(feedback_msg)
            
            if goal_handle.is_cancel_requested:
                #self.get_logger().info('cancel!')
                Data_To_Pub.x=0.0
                Data_To_Pub.y=0.0
                Data_To_Pub.theta=0.0
                self.pub.publish(Data_To_Pub)
                result = ChassisMoveTo.Result()
                result.time = -1.
                return result
            total_time+=dt
            time.sleep(dt)
        self.get_logger().info('------------Move Done!-----------')
        Data_To_Pub = Pose2D()
        Data_To_Pub.x=0.0
        Data_To_Pub.y=0.0
        Data_To_Pub.theta=0.0
        self.pub.publish(Data_To_Pub)
        result = ChassisMoveTo.Result()
        result.time = total_time
        goal_handle.succeed()
        return result

def main():
    rclpy.init() # 初始化rclpy
    node = Chassis_node('chassis_move_server')  # 新建一个节点
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    rclpy.shutdown() # 关闭rclpy