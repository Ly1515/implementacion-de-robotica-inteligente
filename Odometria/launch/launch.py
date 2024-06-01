import os
from launch import LaunchDescription
from launch_ros.actions import Node 
from launch.actions import ExecuteProcess


def generate_launch_description():
    Odometria_Node = Node(package = 'Odometria', executable = 'odometria_node', output = 'screen')

    teleop_node = ExecuteProcess(cmd=['gnome-terminal','--','ros2','run','teleop_twist_keyboard','teleop_twist_keyboard'],
                                 output= 'screen')
	
    RosBag_node = ExecuteProcess(cmd=['gnome-terminal','--','ros2','bag','record','/DatoFinal'], 
                                 output= 'screen')
    

    MicroRos_node = ExecuteProcess(cmd=['gnome-terminal','--','ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '--dev', '/dev/ttyUSB0'], 
                                   output= 'screen')

    l_d = LaunchDescription([Odometria_Node,teleop_node,RosBag_node,MicroRos_node])
    
    return l_d