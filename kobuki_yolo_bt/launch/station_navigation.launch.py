from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodo navegador de estaciones
        Node(
            package='kobuki_yolo_bt',
            executable='station_navigator_node',
            name='station_navigator',
            output='screen'
        ),
        
        # Nodo tfseeker para YOLO
        Node(
            package='tf_seeker',
            executable='tf_seeker_node',
            name='tf_seeker',
            output='screen'
        ),
        
        # Nodo YOLO para detección
        Node(
            package='yolov8_ros',
            executable='yolo_node.py',
            name='yolo_node',
            output='screen',
            parameters=[{
                'model': 'yolov8m.pt',
                'device': 'cuda:0',
                'threshold': 0.5,
                'enable': True
            }]
        )
    ])