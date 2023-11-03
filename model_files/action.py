from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from xxx.action import Xxx_action
class xxx_node(Node):
    def __init__(self,name):
        super().__init__(name)
        self.xxx_server = ActionServer(
            self, Xxx_action, 'xxx', self.xxx_execute
            # ,callback_group=MutuallyExclusiveCallbackGroup()
        )

    def xxx_execute(self,goal_handle: ServerGoalHandle):
        goal = goal_handle.request.xx
        feedback_msg = Xxx_action.Feedback()
        while True:
            #执行
            pass
            #Feedback
            goal_handle.publish_feedback(feedback_msg)
            #中途取消
            if goal_handle.is_cancel_requested:
                #self.get_logger().info('cancel!')
                result = Xxx_action.Result()
                result.x = 0
                return result
            
            #成功结束
            if 1:#结束条件
                goal_handle.succeed()
                result = Xxx_action.Result()
                result.x=0
                return result
