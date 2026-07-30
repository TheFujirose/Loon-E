import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

default_slam_params_file = os.path.join(
    get_package_share_directory('loone'),
    'config',
    'mapper_params_online_async.yaml'
)

default_depth_to_laserscan_params_file = os.path.join(
    get_package_share_directory('loone'),
    'config',
    'depth_to_laserscan.yaml'
)

default_custom_object_detection_config_path = os.path.join(
    get_package_share_directory('loone'),
    'config',
    'model.yaml'
)

# custom_onnx_file in model.yaml is a relative path; override it here with the
# absolute path of the ONNX file as installed into the loone package share
# directory, since the ZED SDK opens the path directly without resolving it
# against the package share directory.
default_model_onnx_path = os.path.join(
    get_package_share_directory('loone'),
    'model',
    'loone.onnx'
)


def generate_launch_description():
    camera_name = LaunchConfiguration('camera_name')
    camera_model = LaunchConfiguration('camera_model')
    zed_node_name = LaunchConfiguration('zed_node_name')
    slam_params_file = LaunchConfiguration('slam_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    sim_mode = LaunchConfiguration('sim_mode')
    sim_address = LaunchConfiguration('sim_address')
    sim_port = LaunchConfiguration('sim_port')

    zed_wrapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('zed_wrapper'),
                'launch',
                'zed_camera.launch.py'
            )
        ),
        launch_arguments={
            'camera_name': camera_name,
            'camera_model': camera_model,
            'node_name': zed_node_name,
            'publish_map_tf': 'false',  # SLAM Toolbox publishes map -> odom instead
            # Our robot_state_publisher owns the whole description now, including the
            # ZED's own frames -- loone_asv.urdf.xacro includes zed_macro and makes the
            # camera the root link. Left at its default `true`, the wrapper starts a
            # SECOND robot_state_publisher for the same camera, which is two competing
            # robot descriptions on the graph.
            'publish_urdf': 'false',
            'use_sim_time': use_sim_time,
            # These three override simulation.* in the submodule's common_stereo.yaml
            # (see zed_camera.launch.py's node_parameters overrides), so sim vs
            # hardware is a launch argument now -- there is no need to switch the
            # zedx submodule between its 'master' and 'simulator' branches.
            'sim_mode': sim_mode,
            'sim_address': sim_address,
            'sim_port': sim_port,
            'custom_object_detection_config_path': default_custom_object_detection_config_path,
            'param_overrides': 'object_detection.custom_onnx_file:=' + default_model_onnx_path,
        }.items()
    )

    # Depth image -> LaserScan, subscribing to the ZED node's registered
    # depth image and re-publishing it as a 2D scan for SLAM Toolbox.
    depth_to_laserscan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        output='screen',
        parameters=[
            default_depth_to_laserscan_params_file,
            {
                'output_frame': [camera_name, '_camera_link'],
                'use_sim_time': use_sim_time,
            }
        ],
        remappings=[
            ('depth', ['/', camera_name, '/', zed_node_name, '/depth/depth_registered']),
            ('depth_camera_info', ['/', camera_name, '/', zed_node_name, '/depth/camera_info']),
            ('scan', '/scan'),
        ]
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )
        ),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': use_sim_time,
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_name',
            default_value='zedx',
            description='Name of the ZED X camera, used as its node namespace.'),
        DeclareLaunchArgument(
            'camera_model',
            default_value='zedx',
            description='Model of the ZED camera passed to the zed_wrapper launch file.'),
        DeclareLaunchArgument(
            'zed_node_name',
            default_value='zed_node',
            description='Name of the zed_wrapper node inside the camera_name namespace.'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=default_slam_params_file,
            description='Path to the SLAM Toolbox parameters YAML file.'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulated /clock time instead of the system clock.'),
        DeclareLaunchArgument(
            'sim_mode',
            default_value='false',
            description='Connect the ZED wrapper to a simulation server instead of a real '
                        'camera. Overrides simulation.sim_enabled in common_stereo.yaml.',
            choices=['true', 'false']),
        DeclareLaunchArgument(
            'sim_address',
            default_value='127.0.0.1',
            description='Address of the Isaac Sim host running the ZED Camera Helper. '
                        'Leave at 127.0.0.1 when Isaac Sim and this stack share a machine; '
                        'set it to the simulator host when they do not.'),
        DeclareLaunchArgument(
            'sim_port',
            default_value='30000',
            description='Port of the ZED streaming server in Isaac Sim '
                        '(ZED_STREAMING_PORT in ros2_bridge.py).'),
        zed_wrapper_launch,
        depth_to_laserscan_node,
        slam_toolbox_launch,
    ])
