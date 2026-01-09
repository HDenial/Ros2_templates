from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    param_config_path = os.path.join(
        get_package_share_directory('my_launch_app'),
        'config','number_config.yaml'
    )
    
    number_publisher = Node(
        package="my_py_pkg", 
        executable="number",
        name="number_publisher",
        remappings=[
            ('/number', '/number_topic')
        ],
        #parameters=[
        #    {'number': 5},
        #    {'timer_period': 2.0}
        #],
        parameters=[param_config_path],
    )

    number_counter = Node(
        package="my_py_pkg",
        executable="counter"
    )

    ld.add_action(number_publisher)
    ld.add_action(number_counter)



    return ld

#example of launch file on python