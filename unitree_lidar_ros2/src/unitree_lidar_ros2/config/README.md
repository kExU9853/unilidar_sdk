# LIO-SAM Configuration for Unitree L1 LiDAR

## Overview
This folder contains configuration files used to integrate LIO-SAM with the Unitree L1 LiDAR in ROS 2.

- `liosam_params.yaml`: Parameters passed to LIO-SAM nodes via the launch file.

The launch file `../launch/liosam_lidar.launch.py` automatically loads this config and starts LIO-SAM nodes with the specified topics and frames.

## Key Features
- **Angle filtering (0–180 degrees)**: Limit LiDAR field of view to a half scan.
- **Topic and frame configuration**: Set input topics and TF frame IDs.
- **Map saving**: Control saving of global map as PCD files.

## Usage
Start LIO-SAM with Unitree L1 integration:

```bash
ros2 launch unitree_lidar_ros2 liosam_lidar.launch.py
```

Optional arguments:
```bash
# Start with the L1 driver
ros2 launch unitree_lidar_ros2 liosam_lidar.launch.py use_driver:=true port:=/dev/ttyUSB0

# Customize topics
ros2 launch unitree_lidar_ros2 liosam_lidar.launch.py \
  cloud_topic:=/custom/cloud imu_topic:=/custom/imu
```

The launch file automatically loads `config/liosam_params.yaml`. Edit that file to change behavior.

## Angle Filtering (Field of View)
To limit the point cloud to 0–180 degrees, adjust the `pointCloudFilter` section in `liosam_params.yaml`:

```yaml
pointCloudFilter:
  enabled: true
  # Angles are in radians
  minAngle: 0.0        # 0 deg
  maxAngle: 3.14159    # 180 deg = pi
```

Other examples:
- 0–90°: `minAngle: 0.0`, `maxAngle: 1.5708`
- 90–270°: `minAngle: 1.5708`, `maxAngle: 4.7124`
- 180–360°: `minAngle: 3.14159`, `maxAngle: 6.2832`

Angle reference:
- 0° = 0 rad
- 90° = π/2 ≈ 1.5708
- 180° = π ≈ 3.14159
- 270° = 3π/2 ≈ 4.7124
- 360° = 2π ≈ 6.2832

## Distance and Height Filters
```yaml
pointCloudFilter:
  minDistance: 0.1   # meters
  maxDistance: 50.0  # meters
  minHeight: -2.0    # meters
  maxHeight: 5.0     # meters
```

## Map Saving
```yaml
savePCD: true
savePCDDirectory: "/tmp/lio_sam_maps"
```

## Troubleshooting
- Ensure the `liosam_params.yaml` file exists at `unitree_lidar_ros2/src/unitree_lidar_ros2/config/`.
- After changing parameters, restart the launch to apply them.
- Check console logs to confirm parameters are being declared/loaded by nodes.
- Verify topic names and frame IDs match your setup.

