from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

GCS_DIR = '/home/benja/Proyectos/tesis'   # carpeta donde está simu.py

def generate_launch_description():
    return LaunchDescription([
        Node(package='ugv_sim', executable='sim_node',
             name='ugv_sim', output='screen'),
        Node(package='ugv_policy', executable='policy_node',
             name='ugv_policy', output='screen'),
        ExecuteProcess(
            cmd=['python3', f'{GCS_DIR}/simu.py'],
            cwd=GCS_DIR, output='screen'),
    ])
