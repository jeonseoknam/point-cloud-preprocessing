"""Standalone launch for the C++ RANSAC ground filter.

Replacement for the earlier Python launch — same topic interface, same
parameter names, but the executable now is the C++ build of this
package. PCL SACSegmentation typically runs in 3–8 ms per scan, well
within the 100 ms real-time budget of a 10 Hz LiDAR.

Example:

    ros2 launch pointcloud_ground_filter_cpp ransac_ground_filter.launch.py

    # Customise
    ros2 launch pointcloud_ground_filter_cpp ransac_ground_filter.launch.py \\
        --ros-args -p distance_threshold:=0.15 -p eps_angle_deg:=20.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    from launch.substitutions import PythonExpression

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_faulty = LaunchConfiguration('use_faulty')
    output_topic = LaunchConfiguration('output_topic')

    # Auto-select input topic from use_faulty so this launch composes
    # naturally with full_pipeline.launch.py's use_lidar_fault flag.
    # Override explicitly via input_topic:= if needed.
    auto_input_topic = PythonExpression([
        "'/lidar/points_faulty' if '", use_faulty,
        "'.lower() == 'true' else '/morai/lidar/points'",
    ])
    input_topic = LaunchConfiguration('input_topic', default=auto_input_topic)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Match the rest of the bag-replay pipeline.'),
        DeclareLaunchArgument(
            'use_faulty', default_value='true',
            description=(
                'If true, subscribe to /lidar/points_faulty (matches the '
                'bringup default of use_lidar_fault=true). If false, '
                'subscribe to the raw /morai/lidar/points. Overridden by '
                'an explicit input_topic:= argument.'
            )),
        DeclareLaunchArgument(
            'input_topic', default_value=auto_input_topic,
            description=(
                'Raw PointCloud2 input to filter. Default tracks '
                'use_faulty automatically.'
            )),
        DeclareLaunchArgument(
            'output_topic', default_value='/lidar/no_ground',
            description='Filtered (ground-removed) PointCloud2 output.'),

        Node(
            package='pointcloud_ground_filter_cpp',
            executable='ransac_ground_filter',
            name='ransac_ground_filter',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'input_topic': input_topic,
                'output_topic': output_topic,
                'diagnostics_topic': '/lidar/ground_filter/diagnostics',

                # PCL SACSegmentation hyper-parameters.
                # Tuned for MORAI VLP-32 driving in K-City: flat road,
                # ~5cm range noise. Tighten distance_threshold (e.g.
                # 0.10) if you want fewer ground points to leak through;
                # widen eps_angle_deg (e.g. 40) if the road has notable
                # grade.
                'max_iter': 50,
                'distance_threshold': 0.20,
                'eps_angle_deg': 30.0,
                'min_inliers': 500,

                # Pre-clip z-band for candidate ground points. Wide
                # default ≈ disabled. Narrow to your sensor's mounting
                # height (e.g. roof-mounted VLP-32 typically sees ground
                # at z ≈ -1.5m to -1.8m in sensor frame) for cleaner
                # plane fits.
                'z_min_for_ransac': -3.0,
                'z_max_for_ransac':  0.5,
            }],
        ),
    ])
