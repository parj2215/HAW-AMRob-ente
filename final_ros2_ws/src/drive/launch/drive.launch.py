from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drive',  # Replace with the name of your package
            executable='ackermann_fsm',  # Replace with the name of your executable/script
            name='ackermann_fsm',
            output='screen',
            parameters=[
                {
                    'serial_port': '/dev/ttyACM0',  # Serial port for Arduino
                    'baud_rate': 9600,            # Baud rate for serial communication
                    'safe_distance': 0.5,         # Minimum safe distance in meters
                    'exploration_speed': 1530,    # PWM for exploration
                    'return_speed': 1530,         # PWM for returning to start
                    'stop_pwm': 1500,             # PWM for stopping
                    'angle_straight': 90,         # Straight angle for the servo
                    'angle_left': 105,            # Left turn angle
                    'angle_right': 85,            # Right turn angle
                    'turn_distance': 3.5,         # Distance for random turns in meters
                    'exploration_time': 900       # Exploration time in seconds (15 minutes)
                }
            ]
        ),
    ])
