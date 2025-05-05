import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Obtener directorios de paquetes
    pkg_dir = get_package_share_directory('kobuki_yolo_bt')
    yolo_pkg = get_package_share_directory('yolo_bringup')  # Asumiendo que existe
    
    # Configuraciones
    use_sim = LaunchConfiguration('use_sim')
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Use simulation (Gazebo) if true. Use real robot if false.')
    
    # Archivos de configuración
    bt_xml_file = os.path.join(pkg_dir, 'config', 'kobuki_fetch_object_bt.xml')
    stations_config_file = os.path.join(pkg_dir, 'config', 'stations.yaml')
    
    # Nodos del proyecto
    voice_command_node = Node(
        package='kobuki_yolo_bt',
        executable='voice_command_node',
        name='voice_command_node',
        output='screen'
    )
    
    object_detection_node = Node(
        package='kobuki_yolo_bt',
        executable='object_detection_node',
        name='object_detection_node',
        output='screen'
    )
    
    behavior_tree_node = Node(
        package='kobuki_yolo_bt',
        executable='behavior_tree_node',
        name='behavior_tree_node',
        output='screen',
        parameters=[{'bt_xml_file': bt_xml_file}]
    )
    
    kobuki_control_node = Node(
        package='kobuki_yolo_bt',
        executable='kobuki_control_node',
        name='kobuki_control_node',
        output='screen'
    )
    
    station_manager_node = Node(
        package='kobuki_yolo_bt',
        executable='station_manager_node',
        name='station_manager_node',
        output='screen',
        parameters=[{'stations_config_file': stations_config_file}]
    )
    
    # Lanzamiento del detector YOLO
    yolo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(yolo_pkg, 'launch', 'yolo.sim.launch.py')
        ),
        condition=LaunchConfiguration('use_sim')
    )
    
    # Crear el LaunchDescription
    ld = LaunchDescription()
    
    # Añadir argumentos
    ld.add_action(use_sim_arg)
    
    # Añadir nodos
    ld.add_action(voice_command_node)
    ld.add_action(object_detection_node)
    ld.add_action(behavior_tree_node)
    ld.add_action(kobuki_control_node)
    ld.add_action(station_manager_node)
    
    # Añadir otras dependencias
    ld.add_action(yolo_launch)
    
    return ld 