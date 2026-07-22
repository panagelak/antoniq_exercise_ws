import os
import tempfile

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, OpaqueFunction
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    row_count = float(LaunchConfiguration('row_count').perform(context))
    row_length = float(LaunchConfiguration('row_length').perform(context))
    row_width = float(LaunchConfiguration('row_width').perform(context))
    wall_thickness = float(LaunchConfiguration('wall_thickness').perform(context))
    wall_height = float(LaunchConfiguration('wall_height').perform(context))
    headland_extension = float(LaunchConfiguration('headland_extension').perform(context))
    obstacle_distance = float(LaunchConfiguration('obstacle_distance').perform(context))
    front_half_angle = float(LaunchConfiguration('front_half_angle').perform(context))
    slam = LaunchConfiguration('slam').perform(context) == 'true'
    nav2 = LaunchConfiguration('nav2').perform(context) == 'true'
    nav_rviz = LaunchConfiguration('nav_rviz').perform(context) == 'true'
    use_ros2_control = LaunchConfiguration('use_ros2_control').perform(context) == 'true'
    mission_status_gui = LaunchConfiguration('mission_status_gui').perform(context) == 'true'

    bringup_share = get_package_share_directory('antoniq_bringup')
    description_share = get_package_share_directory('antoniq_description')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    # Expand the greenhouse world macro into a concrete SDF world gz sim can load.
    world_xacro_path = os.path.join(bringup_share, 'worlds', 'greenhouse.sdf.xacro')
    world_doc = xacro.process_file(
        world_xacro_path,
        mappings={
            'row_count': str(int(row_count)),
            'row_length': str(row_length),
            'row_width': str(row_width),
            'wall_thickness': str(wall_thickness),
            'wall_height': str(wall_height),
            'headland_extension': str(headland_extension),
        },
    )
    generated_world_path = os.path.join(tempfile.gettempdir(), 'antoniq_greenhouse.sdf')
    with open(generated_world_path, 'w') as world_file:
        world_file.write(world_doc.toprettyxml(indent='  '))

    # Same layout formulas as the xacro macro, used here to spawn the robot in the
    # box's near corner, in the headland strip in front of row 0, facing +x into it.
    row_pitch = row_width + wall_thickness
    field_width = row_count * row_pitch + wall_thickness
    y_start = -field_width / 2.0 + wall_thickness / 2.0
    row_0_y = y_start + row_pitch / 2.0
    spawn_x = -(row_length / 2.0)
    spawn_y = row_0_y

    # antoniq_description's URDF carries its own gz-sim sensor and diff-drive plugin
    # tags, so it's spawned straight from robot_description with no model.sdf involved.
    # use_ros2_control selects, inside the xacro, between the ros2_control/gz_ros2_control
    # wheel-drive path and the classic gz-sim DiffDrive/JointStatePublisher plugins.
    robot_xacro_path = os.path.join(description_share, 'urdf', 'waffle.urdf.xacro')
    robot_description = xacro.process_file(
        robot_xacro_path,
        mappings={
            'use_ros2_control': 'true' if use_ros2_control else 'false'
        },
    ).toxml()

    # waffle_bridge.yaml (ros2_control): cmd_vel/odom/tf/joint_states are native ROS 2 topics
    # published/subscribed straight by controller_manager, nothing to bridge.
    # waffle_bridge_classic.yaml (gz-sim DiffDrive plugin): those four live on gz transport and
    # need bridging into ROS.
    bridge_filename = 'waffle_bridge.yaml' if use_ros2_control else 'waffle_bridge_classic.yaml'
    bridge_params_path = os.path.join(bringup_share, 'params', bridge_filename)

    # sdformat_urdf rewrites the URDF's package:// mesh URIs into model://antoniq_description/...
    # when it converts robot_description to SDF, so gz sim needs antoniq_description's share
    # parent on its resource path to resolve them back to files.
    set_gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.dirname(description_share),
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_share, 'launch',
                                                   'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ['-r -v2 ', generated_world_path],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time == 'true',
            'robot_description': robot_description,
        }],
    )
    # Twist Mux
    dir_platform_config = PathJoinSubstitution([FindPackageShare("antoniq_bringup"), 'config'])
    config_twist_mux = ParameterFile([dir_platform_config, '/twist_mux.yaml'], allow_substs=True)
    # diff_drive_base_controller subscribes on its own namespaced ~/cmd_vel (controller_manager
    # runs unnamespaced, so that's /diff_drive_base_controller/cmd_vel) rather than a bare
    # /cmd_vel; the classic gz-sim DiffDrive plugin listens on plain /cmd_vel instead.
    cmd_vel_out_topic = '/diff_drive_base_controller/cmd_vel' if use_ros2_control else '/cmd_vel'
    node_twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        output='screen',
        remappings=[
            ('cmd_vel_out', cmd_vel_out_topic),
            ('/diagnostics', 'diagnostics'),
        ],
        parameters=[config_twist_mux, {
            'use_sim_time': use_sim_time == 'true'
        }],
    )

    spawn_waffle = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name',
            'antoniq_waffle',
            '-topic',
            'robot_description',
            '-x',
            str(spawn_x),
            '-y',
            str(spawn_y),
            '-z',
            '0.01',
        ],
        output='screen',
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_params_path}'],
        output='screen',
    )

    # Only relevant with use_ros2_control:=true. The gz_ros2_control plugin (loaded via
    # waffle.urdf.xacro's <ros2_control>/<gazebo> tags) brings up its own controller_manager
    # inside the gz-sim process once the entity is spawned; these just load/activate controllers
    # on it. They're chained off spawn_waffle's exit (rather than run in parallel) because that
    # controller_manager doesn't exist until the entity does.
    spawn_controllers = None
    if use_ros2_control:
        joint_state_broadcaster_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        )

        diff_drive_base_controller_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_base_controller'],
            output='screen',
        )

        spawn_controllers = RegisterEventHandler(event_handler=OnProcessExit(
            target_action=spawn_waffle,
            on_exit=[joint_state_broadcaster_spawner, diff_drive_base_controller_spawner],
        ))

    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw'],
        output='screen',
    )

    rotate_heading_server = Node(
        package='antoniq_nodes',
        executable='rotate_heading_server_node',
        name='rotate_heading_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time == 'true',
        }],
    )

    obstacle_monitor = Node(
        package='antoniq_nodes',
        executable='obstacle_monitor_node',
        name='obstacle_monitor_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time == 'true',
            'obstacle_distance': obstacle_distance,
            'front_half_angle': front_half_angle,
        }],
    )

    workstation_tf_manager = Node(
        package='antoniq_nodes',
        executable='workstation_tf_manager',
        name='workstation_tf_manager',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time == 'true',
            'row_count': row_count,
            'row_length': row_length,
            'row_width': row_width,
            'wall_thickness': wall_thickness,
        }],
    )

    actions = [
        set_gz_resource_path, gz_sim, robot_state_publisher, spawn_waffle, ros_gz_bridge,
        image_bridge, node_twist_mux, rotate_heading_server, obstacle_monitor,
        workstation_tf_manager
    ]

    if spawn_controllers is not None:
        actions.append(spawn_controllers)

    if mission_status_gui:
        actions.append(
            Node(
                package='antoniq_mission_status_rqt',
                executable='antoniq_mission_status_rqt',
                name='antoniq_mission_status_rqt',
                output='screen',
            ))

    if slam:
        navigation_share = get_package_share_directory('antoniq_navigation')
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(navigation_share, 'launch', 'online_async.launch.py')),
                launch_arguments={'use_sim_time': use_sim_time}.items(),
            ))

    if nav2:
        # launch_amcl stays false: SLAM Toolbox (above) already publishes the map->odom
        # transform and /map, so there's no saved map to localize against here.
        navigation_share = get_package_share_directory('antoniq_navigation')
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(navigation_share, 'launch', 'nav2_amcl.launch.py')),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'launch_nav2': 'true',
                    'launch_amcl': 'false',
                }.items(),
            ))

    if nav_rviz:
        nav2_bringup_share = get_package_share_directory('antoniq_bringup')
        nav_rviz_config = os.path.join(nav2_bringup_share, 'rviz', 'nav2_default_view.rviz')
        actions.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', nav_rviz_config],
                parameters=[{
                    'use_sim_time': use_sim_time == 'true'
                }],
                output='screen',
            ))

    return actions


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('use_sim_time',
                              default_value='true',
                              description='Use the Gazebo /clock topic as ROS time'),
        DeclareLaunchArgument(
            'row_count',
            default_value='4',
            description='Number of parallel crop-row corridors in the greenhouse world'),
        DeclareLaunchArgument('row_length',
                              default_value='2.0',
                              description='Length of each crop row corridor [m]'),
        DeclareLaunchArgument(
            'row_width',
            default_value='0.8',
            description=
            'Actual free/clear width of each crop row corridor, wall face to wall face [m]'),
        DeclareLaunchArgument('wall_thickness',
                              default_value='0.3',
                              description='Thickness of the walls dividing crop rows [m]'),
        DeclareLaunchArgument('wall_height',
                              default_value='0.4',
                              description='Height of the walls dividing crop rows [m]'),
        DeclareLaunchArgument(
            'headland_extension',
            default_value='0.8',
            description=
            'Length the outer boundary walls extend beyond each row end; the enclosed turnaround space at both ends of the box [m]'
        ),
        DeclareLaunchArgument(
            'obstacle_distance',
            default_value='0.7',
            description=
            'obstacle_monitor_node: distance in front of the robot, in meters, within which a laser scan return counts as an obstacle'
        ),
        DeclareLaunchArgument(
            'front_half_angle',
            default_value='0.35',
            description=
            'obstacle_monitor_node: half-width, in radians, of the forward-facing cone checked for obstacles'
        ),
        DeclareLaunchArgument(
            'slam',
            default_value='true',
            description='Launch antoniq_navigation\'s online_async (SLAM Toolbox) mapping node'),
        DeclareLaunchArgument(
            'nav2',
            default_value='true',
            description=
            "Launch antoniq_navigation's nav2 stack (controller/planner/smoother/bt_navigator) so RViz's Nav2 Goal actually has something to talk to"
        ),
        DeclareLaunchArgument(
            'nav_rviz',
            default_value='true',
            description=
            'Launch RViz2 with the nav2 view (map, costmaps, plans) instead of the plain robot view'
        ),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='false',
            description=
            ('true: drive the wheels through ros2_control (gz_ros2_control hardware interface + '
             "diff_drive_controller/joint_state_broadcaster, antoniq_controllers.yaml). false: gz-sim's "
             'built-in DiffDrive/JointStatePublisher system plugins instead, bridged into ROS directly.'
             )),
        DeclareLaunchArgument(
            'mission_status_gui',
            default_value='true',
            description=
            ('Launch antoniq_mission_status_rqt, an rqt GUI plugin showing antoniq_mission_node\'s '
             'current row/waypoint progress (MissionStatus on /mission_status).')),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
