import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_tank_robot = get_package_share_directory('tank_robot')

    tank_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tank_robot, 'launch', 'spawn_robot.launch.py'),
        )
    )

    # 2. Toronyvezérlő (Turret) Node AI-val
    turret_node = Node(
        package='tank_robot_py',
        executable='turret_controller',
        name='turret_controller_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 3. Fegyvervezérlő (Gun) Node
    gun_node = Node(
        package='tank_robot_py',
        executable='gun_controller',
        name='gun_controller_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    #Lövés script
    shoot_node = Node(
        package='tank_robot_py',
        executable='shoot_controller',
        name='shoot_controller_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    launchDescriptionObject = LaunchDescription()
    launchDescriptionObject.add_action(tank_spawn)
    launchDescriptionObject.add_action(turret_node)
    launchDescriptionObject.add_action(gun_node)
    launchDescriptionObject.add_action(shoot_node)
    return launchDescriptionObject