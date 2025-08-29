# follow_wall.launch.py
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    delay_sec = LaunchConfiguration('delay_sec')
    ros_domain_id = LaunchConfiguration('ros_domain_id')
    use_sim_time = LaunchConfiguration('use_sim_time')
    tb3_model = LaunchConfiguration('tb3_model')

    # --- Gazebo: TurtleBot3 Stage1 world ---
    tb3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    stage1_launch = os.path.join(tb3_gazebo_pkg, 'launch', 'turtlebot3_dqn_stage1.launch.py')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(stage1_launch),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # --- Your follower node ---
    follower = Node(
        package='wall_following_bot',
        executable='follower_node',        # console_scripts entry in setup.py should match this
        name='follower_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'obstacle_avoidance': True
        }],
    )

    return LaunchDescription([
        # ---- Launch args ----
        DeclareLaunchArgument('delay_sec', default_value='12',
                              description='Seconds to wait before starting follower_node (time to drop cubes)'),
        DeclareLaunchArgument('ros_domain_id', default_value='7',
                              description='ROS 2 Domain ID to avoid DDS conflicts'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use /clock from Gazebo'),
        DeclareLaunchArgument('tb3_model', default_value='burger',
                              description='TURTLEBOT3 model: burger | waffle | waffle_pi'),

        # ---- Env: robust local comms + TB3 model ----
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', ros_domain_id),
        SetEnvironmentVariable('TURTLEBOT3_MODEL', tb3_model),

        # ---- Start Gazebo world ----
        gazebo,

        # ---- Start your node after a short delay ----
        TimerAction(period=delay_sec, actions=[follower]),
    ])

