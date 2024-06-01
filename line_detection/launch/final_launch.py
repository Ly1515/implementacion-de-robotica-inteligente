import os
from launch import LaunchDescription
from launch_ros.actions import Node 


def generate_launch_description():
    line_detection_node = Node(package = 'line_detection', executable = 'line_detection_node', output = 'screen')

    color_identification_node = Node(package = 'line_detection', executable = 'color_identification_node', output = 'screen')

    top_level_node = Node(package = 'line_detection', executable = 'top_level_node', output = 'screen')

    l_d = LaunchDescription([line_detection_node, color_identification_node, top_level_node ])
    
    return l_d