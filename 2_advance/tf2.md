# TF2

## TF2 CLI

### 发布B到C的位姿

父坐标系B 子坐标系C
``` bash
ros2 run tf2_ros static_transform_publisher 0 0 3 0 0 3.14 B C
```
    

### 发布B到C的位姿
``` bash
    ros2 run tf2_ros tf2_echo B P
```
### 监听/获取TF关系
```bash
    ros2 run tf2_ros tf2_echo B P
```

## python实现

### 静态广播
```python
from tf2_ros import StaticTransformBroadcaster
    #node中
        #创建静态tf pub
        static_tf_publisher = StaticTransformBroadcaster(self)
        T = TransformStamped()
        #时间
        T.header.stamp = self.get_clock().now().to_msg()
        #坐标系
        T.header.frame_id = A
        T.child_frame_id = B
        #平移
        T.transform.translation.x = float(tx)
        T.transform.translation.y = float(ty)
        T.transform.translation.z = float(tz)
        #旋转四元数
        T.transform.rotation.x = q[0]
        T.transform.rotation.y = q[1]
        T.transform.rotation.z = q[2]
        T.transform.rotation.w = q[3]
        static_tf_publisher.sendTransform(T)
```
    

### 动态广播
```python
from tf2_ros import TransformBroadcaster

class TF_Publisher(Node):
    def __init__(self):
        super().__init__('tf_publisher')

        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.tf_publisher = TransformBroadcaster(self)


    def timer_callback(self):
        T = TransformStamped()
        T.header.stamp = self.get_clock().now().to_msg()
        T.header.frame_id = A
        T.child_frame_id = B
        T.transform.translation.x = m[0][3]
        T.transform.translation.y = m[1][3]
        T.transform.translation.z = m[2][3]
        r = Rotation.from_matrix(m[:3, :3])
        q = r.as_quat()
        T.transform.rotation.w = q[3]
        T.transform.rotation.x = q[0]
        T.transform.rotation.y = q[1]
        T.transform.rotation.z = q[2]
        self.tf_publisher.sendTransform(T)

```

