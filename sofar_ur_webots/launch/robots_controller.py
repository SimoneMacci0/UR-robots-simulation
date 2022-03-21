from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   return LaunchDescription([
       Node(
           package='sofar_ur_webots',
           namespace='ur1',
           executable='ur_controller',
           name='ur1_controller',
           parameters=[
            {"robot_name": "UR5_1"},
           ]
       ),
       Node(
           package='sofar_ur_webots',
           namespace='ur2',
           executable='ur_controller',
           name='ur2_controller',
           parameters=[
            {"robot_name": "UR5_2"},
           ]
       ),
   ])
