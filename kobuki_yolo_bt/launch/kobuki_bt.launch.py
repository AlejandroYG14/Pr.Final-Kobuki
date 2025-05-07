from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declarar argumentos
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    
    # Obtener directorios de paquetes
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    kobuki_yolo_bt_dir = get_package_share_directory('kobuki_yolo_bt')
    
    # Incluir launch de Nav2
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_file,
        }.items()
    )
    
    # Nodo del Behavior Tree
    behavior_tree_node = Node(
        package='kobuki_yolo_bt',
        executable='behavior_tree_node',
        name='behavior_tree_node',
        output='screen'
    )
    
    # Nodo de control de Kobuki
    kobuki_control_node = Node(
        package='kobuki_yolo_bt',
        executable='kobuki_control_node',
        name='kobuki_control_node',
        output='screen'
    )
    
    # Nodo del gestor de estaciones
    station_manager_node = Node(
        package='kobuki_yolo_bt',
        executable='station_manager_node',
        name='station_manager_node',
        output='screen'
    )
    
    return LaunchDescription([
        # Argumentos
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(kobuki_yolo_bt_dir, 'config', 'map.yaml'),
            description='Path to map file'
        ),
        
        # Nodos
        nav2_launch,
        behavior_tree_node,
        kobuki_control_node,
        station_manager_node
    ]) 