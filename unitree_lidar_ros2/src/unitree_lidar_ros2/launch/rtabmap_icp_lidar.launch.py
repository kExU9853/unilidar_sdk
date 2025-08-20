from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # Launch arguments
    use_driver = LaunchConfiguration("use_driver")
    port = LaunchConfiguration("port")
    cloud_topic = LaunchConfiguration("cloud_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    base_frame = LaunchConfiguration("base_frame")
    odom_frame = LaunchConfiguration("odom_frame")

    # Unitree L1 driver (optional)
    driver_node = Node(
        package="unitree_lidar_ros2",
        executable="unitree_lidar_ros2_node",
        name="unitree_lidar_ros2_node",
        output="screen",
        condition=IfCondition(use_driver),
        parameters=[
            {"port": port},
            {"rotate_yaw_bias": 0.0},
            {"range_scale": 0.001},
            {"range_bias": 0.0},
            {"range_max": 50.0},
            {"range_min": 0.0},
            {"cloud_frame": "unilidar_lidar"},
            {"cloud_topic": cloud_topic},
            {"cloud_scan_num": 18},
            {"imu_frame": "unilidar_imu"},
            {"imu_topic": imu_topic},
        ],
    )

    # rtabmap ICP odometry
    icp_odom_node = Node(
        package="rtabmap_odom",
        executable="icp_odometry",
        name="icp_odometry",
        output="screen",
        parameters=[
            {"frame_id": base_frame},
            {"approx_sync": True},
        ],
        remappings=[
            ("scan_cloud", cloud_topic),
            ("imu", imu_topic),
        ],
    )

    # rtabmap mapping (subscribe to odom and/or point cloud)
    rtabmap_node = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=[
            {"frame_id": base_frame},
            {"odom_frame_id": odom_frame},
            {"subscribe_scan_cloud": True},
            {"queue_size": 10},
        ],
        remappings=[
            ("scan_cloud", cloud_topic),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_driver",
                default_value="false",
                description="Whether to also start Unitree L1 driver in this launch",
            ),
            DeclareLaunchArgument(
                "port",
                default_value="/dev/ttyUSB0",
                description="Serial port of Unitree L1 (used only when use_driver=true)",
            ),
            DeclareLaunchArgument(
                "cloud_topic",
                default_value="/unilidar/cloud",
                description="Input point cloud topic from Unitree L1",
            ),
            DeclareLaunchArgument(
                "imu_topic",
                default_value="/unilidar/imu",
                description="Input IMU topic from Unitree L1",
            ),
            DeclareLaunchArgument(
                "base_frame",
                default_value="base_link",
                description="Robot base frame id",
            ),
            DeclareLaunchArgument(
                "odom_frame",
                default_value="odom",
                description="Odometry frame id",
            ),
            driver_node,
            icp_odom_node,
            rtabmap_node,
        ]
    )


