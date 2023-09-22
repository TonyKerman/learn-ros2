
'''来自ros2官方文档的客户端代码,
发送a,b让服务端计算a+b,
使用命令行参数给a,b赋值
感觉不如另一个简洁'''
#使用sys 来获取 命令行参数
import sys
#导入服务类型: AddTwoInts
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        #等待服务端上线
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        #定义请求变量
        self.req = AddTwoInts.Request()

    def send_request(self):
        '''发送请求的函数,'''
        self.req.a = int(sys.argv[1])
        self.req.b = int(sys.argv[2])
        #发送请求,future是接送结果的
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    minimal_client.send_request()

    while rclpy.ok():
        #开始一次
        rclpy.spin_once(minimal_client)
        #如果好了
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                minimal_client.get_logger().info(
                    'Result of add_two_ints: for %d + %d = %d' %
                    (minimal_client.req.a, minimal_client.req.b, response.sum))
            break
    #关闭
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
