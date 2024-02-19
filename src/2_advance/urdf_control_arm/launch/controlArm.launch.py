
import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    #注意要改
    package_name = 'urdf_control_arm'
    urdf_name = "arm.urdf"

    ld = LaunchDescription()
    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')


    #joint_state_publisher_gui 负责发布机器人关节数据信息，通过joint_states话题发布
    #如果要自己控制机器人，需要发布关节数据，这个节点要改为自己的节点
    joint_state_publisher_node = Node(
        package=package_name,
        executable='state_publisher',
        #name='state_publisher',
        arguments=[urdf_model_path]
        )

    #robot_state_publisher_node负责发布机器人模型信息robot_description，并将joint_states数据转换tf信息发布
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_model_path]
        )
    
    #rviz2_node负责显示机器人的信息
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        )

    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(rviz2_node)

    return ld
