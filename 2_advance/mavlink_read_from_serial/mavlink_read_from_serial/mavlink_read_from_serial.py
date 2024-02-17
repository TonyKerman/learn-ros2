import rclpy
from pymavlink import mavutil
from rclpy.node import Node
import serial.tools.list_ports

class MavlinkSerialNode(Node):
    def __init__(self,name):
        super().__init__(name)

        self.ports = list(serial.tools.list_ports.comports())
        self.mav_connection = mavutil.mavlink_connection(self.ports[0][0],baud=115200,dialect='common') 
        self.get_logger().info(f'conneted to {self.ports[0][0]}')
        #self.rate = self.create_rate(100)
        
        while True:
            msg = self.mav_connection.recv_match(blocking = True)
            self.get_logger().info(msg.get_type())
            
            #self.rate.sleep()

    
    def read(self):
        pass
        
        #if msg != None:
        #    self.get_logger().info(str(msg))

def main(args = None):
    rclpy.init(args = args)
    node = MavlinkSerialNode('MavlinkSerialNode')
    rclpy.spin(node)
    rclpy.shutdown() # 关闭rclpy

