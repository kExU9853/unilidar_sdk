from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description() -> LaunchDescription:
    # Launch arguments
    use_driver = LaunchConfiguration("use_driver")
    port = LaunchConfiguration("port")
    cloud_topic = LaunchConfiguration("cloud_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    base_frame = LaunchConfiguration("base_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    map_frame = LaunchConfiguration("map_frame")
    lidar_frame = LaunchConfiguration("lidar_frame")
    imu_frame = LaunchConfiguration("imu_frame")

    # Optional Unitree driver
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
            {"cloud_frame": lidar_frame},
            {"cloud_topic": cloud_topic},
            {"cloud_scan_num": 18},
            {"imu_frame": imu_frame},
            {"imu_topic": imu_topic},
        ],
    )

    # Static transform publishers for coordinate frames
    # Assuming LiDAR and IMU are mounted at the same position as base_link
    # You may need to adjust these transforms based on your actual mounting
    lidar_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="lidar_tf_publisher",
        arguments=[
            "0.0", "0.0", "0.0",  # x, y, z translation
            "0.0", "0.0", "0.0",  # roll, pitch, yaw rotation
            base_frame, lidar_frame  # parent frame, child frame
        ],
        output="screen"
    )

    imu_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="imu_tf_publisher",
        arguments=[
            "0.0", "0.0", "0.0",  # x, y, z translation
            "0.0", "0.0", "0.0",  # roll, pitch, yaw rotation
            base_frame, imu_frame  # parent frame, child frame
        ],
        output="screen"
    )

    # Map to odom static transform (initial)
    map_odom_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_odom_tf_publisher",
        arguments=[
            "0.0", "0.0", "0.0",  # x, y, z translation
            "0.0", "0.0", "0.0",  # roll, pitch, yaw rotation
            map_frame, odom_frame  # parent frame, child frame
        ],
        output="screen"
    )

    # LIO-SAM nodes - using the actual executable names
    # Load LIO-SAM configuration from local config file
    liosam_config_path = os.path.join(
        os.path.dirname(__file__), 
        "..", 
        "config", 
        "liosam_params.yaml"
    )
    
    liosam_imu_node = Node(
        package="lio_sam",
        executable="lio_sam_imuPreintegration",
        name="lio_sam_imuPreintegration",
        output="screen",
        parameters=[
            liosam_config_path,
            {"pointCloudTopic": cloud_topic},
            {"imuTopic": imu_topic},
            {"lidarFrame": lidar_frame},
            {"imuFrame": imu_frame},
            {"odomFrame": odom_frame},
            {"mapFrame": map_frame},
            {"baseLinkFrame": base_frame},
        ],
        remappings=[
            ("/points_raw", cloud_topic),
            ("/imu", imu_topic),
        ],
    )

    liosam_image_node = Node(
        package="lio_sam",
        executable="lio_sam_imageProjection",
        name="lio_sam_imageProjection",
        output="screen",
        parameters=[
            liosam_config_path,
            {"pointCloudTopic": cloud_topic},
            {"imuTopic": imu_topic},
            {"lidarFrame": lidar_frame},
            {"imuFrame": imu_frame},
            {"odomFrame": odom_frame},
            {"mapFrame": map_frame},
            {"baseLinkFrame": base_frame},
        ],
        remappings=[
            ("/points_raw", cloud_topic),
            ("/imu", imu_topic),
        ],
    )

    liosam_feature_node = Node(
        package="lio_sam",
        executable="lio_sam_featureExtraction",
        name="lio_sam_featureExtraction",
        output="screen",
        parameters=[
            liosam_config_path,
            {"pointCloudTopic": cloud_topic},
            {"imuTopic": imu_topic},
            {"lidarFrame": lidar_frame},
            {"imuFrame": imu_frame},
            {"odomFrame": odom_frame},
            {"mapFrame": map_frame},
            {"baseLinkFrame": base_frame},
        ],
        remappings=[
            ("/points_raw", cloud_topic),
            ("/imu", imu_topic),
        ],
    )

    liosam_map_node = Node(
        package="lio_sam",
        executable="lio_sam_mapOptimization",
        name="lio_sam_mapOptimization",
        output="screen",
        parameters=[
            liosam_config_path,
            {"pointCloudTopic": cloud_topic},
            {"imuTopic": imu_topic},
            {"lidarFrame": lidar_frame},
            {"imuFrame": imu_frame},
            {"odomFrame": odom_frame},
            {"mapFrame": map_frame},
            {"baseLinkFrame": base_frame},
        ],
        remappings=[
            ("/points_raw", cloud_topic),
            ("/imu", imu_topic),
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
            DeclareLaunchArgument(
                "map_frame",
                default_value="map",
                description="Map frame id",
            ),
            DeclareLaunchArgument(
                "lidar_frame",
                default_value="unilidar_lidar",
                description="Lidar frame id",
            ),
            DeclareLaunchArgument(
                "imu_frame",
                default_value="unilidar_imu",
                description="IMU frame id",
            ),
            # Static transform publishers
            lidar_tf_node,
            imu_tf_node,
            map_odom_tf_node,
            # LIO-SAM nodes
            driver_node,
            liosam_imu_node,
            liosam_image_node,
            liosam_feature_node,
            liosam_map_node,
        ]
    )


