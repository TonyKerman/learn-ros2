// Some code
```python
import rclpy
from rclpy.node import Node
# 1.导入消息类型JointState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation


class XxxNode(Node):
    def __init__(self,name):
        super().__init__(name)

        self.joint_states_publisher_ =     self.create_publisher(JointState,"joint_states", 10)
        self.tf_publisher = TransformBroadcaster(self)
        self._static_tf_pub('mpu_base','base_link',Rotation.from_euler('zyx',(np.pi/4,np.pi/2,0)))
        self.joint_states = JointState()
 
        

        self._joint_state_msg_init()         
        if self._serial_init():
            exit()

        
        self.pub_rate = self.create_rate(10)
        self.thread_pub = threading.Thread(target=self._thread_pub)
       
        self.thread_pub.setDaemon(True)
 
        self.get_logger().info(f'got {msg.get_type()}')
        self.thread_pub.start()
        self.joint_states.header.stamp = self.get_clock().now().to_msg()
    
        

            
    def quat_to_TF(self, A: str, B: str, tx: float, ty: float, tz: float, q: list):
        if len(q) != 4:
            raise IndexError('Quaternion must be a list of 4 elements')

        T = TransformStamped()
        T.header.stamp = self.get_clock().now().to_msg()
        T.header.frame_id = A
        T.child_frame_id = B
        T.transform.translation.x = float(tx)
        T.transform.translation.y = float(ty)
        T.transform.translation.z = float(tz)
        T.transform.rotation.x = q[0]
        T.transform.rotation.y = q[1]
        T.transform.rotation.z = q[2]
        T.transform.rotation.w = q[3]
        return T
    
    def _joint_state_msg_init(self) -> None:
        self.get_logger().info('publisher start')
        self.joint_states.name = ['joint1','joint2','joint3','joint4','joint5','joint6']
        self.joint_states.position = np.zeros(6,dtype=float).tolist()
        self.joint_states.velocity = []
        #self.joint_states.name = ['joint1']
        #self.joint_states.position = [10.0]
        #self.joint_states.velocity = [0.0]
        self.joint_states.effort = []
        self.joint_states.header.frame_id = ""
    
    def _static_tf_pub(self,A:str='world1',B:str='world2',R:Rotation=Rotation.from_matrix(np.eye(3))):
        static_tf_publisher = StaticTransformBroadcaster(self)
        static_tf_publisher.sendTransform(
            self.quat_to_TF(A,B, 0, 0, 0, R.as_quat()))

 
def main(args=None):
    rclpy.init(args=args) # 初始化rclpy
    node = XxxNode("xxx")  # 新建一个节点dd
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy

``` 