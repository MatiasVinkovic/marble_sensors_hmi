#!/usr/bin/env python3
"""
Lance l'IHM MARBLE Sensors Monitor.

  ros2 launch marble_sensors_hmi sensors_hmi.launch.py

C'est l'IHM qui lance elle-même les nodes capteurs : pour chaque capteur,
choisir le port et le baud dans son panneau puis cliquer "Connecter".
(Plus besoin d'arguments de port ici — tout se règle dans la fenêtre.)
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='marble_sensors_hmi',
            executable='hmi_node',
            name='sensors_hmi',
            output='screen',
        ),
    ])
