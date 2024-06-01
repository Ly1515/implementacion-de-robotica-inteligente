import os
from launch import LaunchDescription
from launch_ros.actions import Node 
from launch.actions import ExecuteProcess


def generate_launch_description():
    video_source = ExecuteProcess(cmd=['gnome-terminal','--','ros2','bag','record','/DatoFinal'], 
                                 output= 'screen')
    video_source = Node(package = 'Odometria', executable = 'odometria_node', output = 'screen')

    

    l_d = LaunchDescription()
    
    return l_d  