"""Top-level bringup for the nav2 + ros2_control "chained controls" stack.

Starts, in order:
  1. slam_launch.py         - ZED wrapper + depth->laserscan + SLAM Toolbox
                              (provides map->odom->zedx_camera_link TF, /odom, /scan).
  2. robot_state_publisher  - publishes base_link + prop/rudder frames from the URDF.
  3. static_transform_pub   - connects zedx_camera_link -> base_link (the camera is the
                              tracked frame; see loone_asv.urdf.xacro for why base_link
                              hangs below it). That URDF does not declare a camera link
                              of its own, so there is nothing else to conflict with the
                              live zedx_camera_link frame this static transform bridges.
  4. controller_manager     - ros2_control node hosting the hardware + controllers.
  5. spawners               - joint_state_broadcaster + asv_forward_controller.
  6. thrust_mixer           - /cmd_vel -> /asv_forward_controller/commands.
  7. busio_node             - /asv/joint_commands -> PCA9685 over I2C; also publishes
                              raw INA3221 battery voltages on battery_raw.
  8. battery_node           - battery_raw -> proper sensor_msgs/BatteryState topics.
  9. phone                  - phone GPS (ADB) -> /navsatfix, fused into the ZED's
                              pos_tracking via gnss_fusion (config/common_stereo.yaml).
  9b. navsat_transform      - GPS<->map/odom conversion utility (config/navsat_transform.yaml).
                              Not part of the TF/localization chain; exists only for the
                              /fromLL, /toLL services a future GPS-waypoint mission node needs.
 10. navigation_launch.py   - nav2 planner/controller/costmaps -> /cmd_vel.

The old task/motor/path_planning nodes are intentionally NOT started here.
Send goals with RViz "2D Goal Pose" or a NavigateToPose action client.

Usage:
    ros2 launch loone bringup.launch.py
    # bench test without a camera/hardware: see the plan's verification section.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    loone_share = get_package_share_directory('loone')
    urdf_share = get_package_share_directory('le1000_urdf_t4')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    camera_name = LaunchConfiguration('camera_name')
    camera_model = LaunchConfiguration('camera_model')
    zed_node_name = LaunchConfiguration('zed_node_name')
    use_sim_time = LaunchConfiguration('use_sim_time')
    sim = LaunchConfiguration('sim')
    sim_address = LaunchConfiguration('sim_address')
    sim_port = LaunchConfiguration('sim_port')

    # Expand the xacro once and share the result with rsp + controller_manager.
    # Robot description lives in the le1000_urdf_t4 submodule (src/loone_urdf),
    # not this package -- see its urdf/loone_asv.urdf.xacro. (Using the minimal
    # loone_asv description for now instead of the CAD-derived le1000_urdf_t4 one;
    # swap this filename back to switch.)
    xacro_path = os.path.join(urdf_share, 'urdf', 'loone_asv.urdf.xacro')
    # camera_name/camera_model are forwarded into the xacro because it now includes
    # the ZED wrapper's own zed_macro and instantiates the camera as the robot's ROOT
    # link. The names must agree with what the wrapper publishes -- expand the xacro
    # with a different camera_name than the wrapper uses and the two halves of the
    # TF tree simply never join up.
    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_path,
                     ' camera_name:=', camera_name,
                     ' camera_model:=', camera_model]),
            value_type=str)
    }

    ros2_control_params = os.path.join(loone_share, 'config', 'ros2_control.yaml')
    nav2_params = os.path.join(loone_share, 'config', 'nav2_params.yaml')

    # 1. Perception / localization (already working on hardware).
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(loone_share, 'launch', 'slam_launch.py')),
        launch_arguments={
            'camera_name': camera_name,
            'camera_model': camera_model,
            'zed_node_name': zed_node_name,
            'use_sim_time': use_sim_time,
            # `sim` drives the ZED wrapper's simulation mode as well as the
            # actuator swap below, so the two can never disagree.
            'sim_mode': sim,
            'sim_address': sim_address,
            'sim_port': sim_port,
        }.items()
    )

    # 2. Robot description -> TF for base_link and the prop/rudder links.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    # 3. (removed) There is no longer a static_transform_publisher bridging the camera
    #    to base_link. loone_asv.urdf.xacro now instantiates the ZED wrapper's own
    #    zed_macro as the robot's ROOT link and carries the camera->base_link offset
    #    as the `cam_1_to_base_link` joint, so robot_state_publisher emits it and the
    #    mount geometry lives in exactly one place. slam_launch.py passes
    #    publish_urdf:=false so the wrapper does not start a competing
    #    robot_state_publisher for the same camera.

    # 4. controller_manager: hosts topic_based_ros2_control hardware + the controllers.
    #    NOTE (Humble): robot_description is passed as a parameter here. On Iron/Jazzy
    #    the controller_manager instead reads it from the /robot_description topic.
    #    use_sim_time comes LAST so it wins over the `false` baked into
    #    ros2_control.yaml. Without this override the controller_manager runs its
    #    update loop on the wall clock while every other node is on /clock, so
    #    command and state stamps disagree by however far sim time has drifted
    #    from real time -- which surfaces as jittery or ignored commands rather
    #    than as an obvious clock error.
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[robot_description, ros2_control_params,
                    {'use_sim_time': use_sim_time}],
    )

    # 5. Spawn the broadcaster + forward controller into the controller_manager.
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    asv_forward_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['asv_forward_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # 6. Control mixing (cmd_vel -> servo fractions).
    thrust_mixer = Node(
        package='loone',
        executable='thrust_mixer',
        name='thrust_mixer',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # 7. Hardware driver (fractions -> PCA9685; also reads the INA3221). Only node touching I2C.
    #    Excluded in sim: it imports board/busio at module scope, so it cannot even
    #    start on a machine without an I2C bus.
    busio_node = Node(
        package='loone',
        executable='busio_node',
        name='busio_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=UnlessCondition(sim),
    )

    # 7b. Simulation stand-in for the driver's open-loop state echo. Isaac Sim
    #     consumes /asv/joint_commands (see isaac_sim/.../ros2_bridge.py), but
    #     nothing would publish /asv/joint_states back to topic_based_ros2_control
    #     without this -- the state interfaces would sit at NaN.
    sim_state_echo = Node(
        package='loone',
        executable='sim_state_echo',
        name='sim_state_echo',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(sim),
    )

    # 8. Battery telemetry (battery_raw -> sensor_msgs/BatteryState). Depends on
    #    busio_node's INA3221 readings, so it is excluded in sim for the same reason.
    battery_node = Node(
        package='loone',
        executable='battery_node',
        name='battery_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=UnlessCondition(sim),
    )

    # 9. Phone GPS bridge: publishes NavSatFix on /navsatfix, which the ZED wrapper's
    #    gnss_fusion (config/common_stereo.yaml) subscribes to and fuses into pos_tracking.
    #    Excluded in sim for the same reason as busio_node: it shells out to `adb` at
    #    startup and dies with FileNotFoundError where no phone/adb is attached.
    #    Isaac Sim supplies pose directly, so there is nothing for it to contribute.
    phone = Node(
        package='loone',
        executable='phone',
        name='phone',
        output='screen',
        condition=UnlessCondition(sim),
    )

    # 9a. Simulation stand-in for phone.py's GPS. navsat_transform will not latch a
    #     datum or answer /fromLL without a fix, so without this task1.launch.py
    #     hangs on "Waiting for /fromLL service" in sim. Derives lat/lon from the
    #     simulated odometry -- see sim_gnss.py for why this is not done inside
    #     Isaac Sim (no rclpy in Kit).
    sim_gnss = Node(
        package='loone',
        executable='sim_gnss',
        name='sim_gnss',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'odom_topic': ['/', camera_name, '/', zed_node_name, '/odom'],
        }],
        condition=IfCondition(sim),
    )

    # 9b. GPS<->map/odom conversion utility -- NOT part of the TF/localization chain
    #     (see config/navsat_transform.yaml). Exposes /fromLL and /toLL so a future
    #     mission node can turn a list of lat/lon waypoints into Nav2 goals.
    navsat_transform_params = os.path.join(loone_share, 'config', 'navsat_transform.yaml')
    navsat_transform = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[navsat_transform_params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('gps/fix', '/navsatfix'),
            # TODO(team): verify against the ZED wrapper's actual topic names if you
            # change the camera_name/zed_node_name launch args (defaults: zedx/zed_node).
            ('imu/data', ['/', camera_name, '/', zed_node_name, '/imu/data']),
            ('odometry/filtered', ['/', camera_name, '/', zed_node_name, '/odom']),
        ],
    )

    # 10. nav2 (produces /cmd_vel). No AMCL/map_server -- SLAM Toolbox owns those.
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
            'autostart': 'true',
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('camera_name', default_value='zedx',
                              description='ZED camera name / namespace (sets the <name>_camera_link frame).'),
        DeclareLaunchArgument('camera_model', default_value='zedx',
                              description='ZED camera model passed to the wrapper.'),
        DeclareLaunchArgument('zed_node_name', default_value='zed_node',
                              description='ZED wrapper node name inside the camera namespace.'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use /clock simulated time. Keep false on the real boat.'),
        DeclareLaunchArgument('sim', default_value='false',
                              description='Simulation mode: swap busio_node for sim_state_echo '
                                          '(Isaac Sim drives the actuators), drop the phone/battery '
                                          'nodes, and point the ZED wrapper at the simulator. '
                                          'Usually set together with use_sim_time:=true.'),
        DeclareLaunchArgument('sim_address', default_value='127.0.0.1',
                              description='Host running Isaac Sim, as seen from this machine. '
                                          'The ZED X is a Jetson-only camera, so this stack runs '
                                          'on the Jetson while Isaac Sim runs on the GPU box -- '
                                          'in that split this must be the GPU box\'s LAN address, '
                                          'and ros2_bridge.py needs ZED_USE_IPC = False.'),
        DeclareLaunchArgument('sim_port', default_value='30000',
                              description='ZED streaming port in Isaac Sim (ZED_STREAMING_PORT).'),
        slam_launch,
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        asv_forward_controller_spawner,
        thrust_mixer,
        busio_node,
        sim_state_echo,
        battery_node,
        phone,
        sim_gnss,
        navsat_transform,
        nav2_launch,
    ])
