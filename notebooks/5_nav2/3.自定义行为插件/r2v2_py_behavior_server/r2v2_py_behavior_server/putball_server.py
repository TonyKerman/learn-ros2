import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rc2024_interfaces.action import PutBall
from std_msgs.msg import UInt32
from geometry_msgs.msg import Pose2D
from time import sleep

class PutBallServer(Node):
    def __init__(self,name):
        super().__init__(name)
        self.action_server_ = ActionServer(self, 
                                        PutBall, 
                                        'putball', 
                                        handle_accepted_callback=self.handle_accepted_callback,
                                        execute_callback=self.execute_callback,
                                        goal_callback=self.goal_callback,
                                        cancel_callback=self.cancel_callback,)
                                        #callback_group=ReentrantCallbackGroup())
        self._goal_handle = None
        self.chassis_pub_ = self.create_publisher(Pose2D,"car/cmd_vel",2)
        self.up_pub_ = self.create_publisher(UInt32,"up_cmd",2)
        self.up_cmd = {'reset':1,'chase_ball':2,'catch_ball':4,'put_ball':8, 'change_angle': 16}


    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT
    def handle_accepted_callback(self, goal_handle):
        """Provide a handle to an accepted goal.当一个目标被接受后，这个方法被调用。"""
        self.get_logger().info('execution...')
        self._goal_handle = goal_handle
        self._goal_handle.execute()

    def execute_callback(self, goal_handle):
        # self.up_pub_.publish(UInt32(data=self.up_cmd['chase_ball']))
        # sleep(1)
        # self.up_pub_.publish(UInt32(data=self.up_cmd['catch_ball']))
        # sleep(2)
        self.up_pub_.publish(UInt32(data=self.up_cmd['put_ball']))
        self.get_logger().info(f'up_CMD put_ball:{self.up_cmd["put_ball"]}')
        sleep(1)
        self.up_pub_.publish(UInt32(data=self.up_cmd['reset']))
        goal_handle.succeed()
        result = PutBall.Result()
        return result
    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info(f'Received cancel request {id(goal_handle)}')
        return CancelResponse.ACCEPT
    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    putball_server = PutBallServer('putball_server')
    rclpy.spin(putball_server)
    putball_server.destroy()
    rclpy.shutdown()