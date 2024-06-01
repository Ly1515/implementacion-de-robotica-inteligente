import os
from launch import LaunchDescription
from launch_ros.actions import Node 


def generate_launch_description():
    line_and_color_detection_node = Node(package = 'final_project', executable = 'line_and_color_detection_node', output = 'screen')

    top_level_node = Node(package = 'final_project', executable = 'top_level_node', output = 'screen')
    
    signal_detection_node = Node(package = 'final_project', executable = 'signal_detection_node', output = 'screen')
    

    l_d = LaunchDescription([line_and_color_detection_node, signal_detection_node, top_level_node])
    
    return l_d
