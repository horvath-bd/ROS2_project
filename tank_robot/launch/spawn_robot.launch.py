import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    pkg_tank_robot = get_package_share_directory('tank_robot')

    gazebo_models_path, ignore_last_dir = os.path.split(pkg_tank_robot)
    os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz.'
    )

    world_arg = DeclareLaunchArgument(
        'world', default_value='world.sdf',
        description='Name of the Gazebo world file to load'
    )

    model_arg = DeclareLaunchArgument(
        'model', default_value='enemy_photo.urdf',
        description='Name of the URDF description to load'
    )

    # Define the path to your URDF or Xacro file
    urdf_file_path = PathJoinSubstitution([
        pkg_tank_robot,  # Replace with your package name
        "urdf",
        LaunchConfiguration('model')  # Replace with your URDF or Xacro file
    ])

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tank_robot, 'launch', 'world.launch.py'),
        ),
        launch_arguments={
        'world': LaunchConfiguration('world'),
        }.items()
    )

    # Launch rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_tank_robot, 'rviz', 'rviz.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': True},
        ]
    )

    # Spawn the URDF model using the `/world/<world_name>/create` service
    spawn_urdf_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "my_tank",
            "-topic", "robot_description",
            "-x", "0.0", "-y", "0.0", "-z", "0.5", "-Y", "1.07"  # Initial spawn position
        ],
        output="screen",
        parameters=[
            {'use_sim_time': True},
        ]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            # Ezt a sort módosítottuk!
            {'robot_description': ParameterValue(Command(['xacro', ' ', urdf_file_path]), value_type=str),
             'use_sim_time': True},
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    #joint_state_publisher_gui_node = Node(
    #    package='joint_state_publisher_gui',
    #    executable='joint_state_publisher_gui',
    #)

    # Híd a Gazebo sim time és a ROS 2 óra között
    clock_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Óra szinkronizáció
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Joint állapotok (Gazebo -> ROS)
            '/world/empty/model/my_tank/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            # Torony vezérlés (ROS -> Gazebo)
            '/model/my_tank/joint/turret_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            # Ágyú vezérlés (ROS -> Gazebo)
            '/model/my_tank/joint/gun_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            #lánctalp vezérlés
            '/model/my_tank/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        ],
        # Mivel a joint_state topic neve Gazebóban specifikus, áttérképezzük a standard ROS névre:
        remappings=[
            ('/world/empty/model/my_tank/joint_state', '/joint_states'),
        ],
        output='screen'
    )

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(rviz_launch_arg)
    launchDescriptionObject.add_action(world_arg)
    launchDescriptionObject.add_action(model_arg)
    launchDescriptionObject.add_action(world_launch)
    launchDescriptionObject.add_action(rviz_node)
    launchDescriptionObject.add_action(spawn_urdf_node)
    launchDescriptionObject.add_action(robot_state_publisher_node)
    #launchDescriptionObject.add_action(joint_state_publisher_gui_node)

    launchDescriptionObject.add_action(clock_bridge_node)

    return launchDescriptionObject