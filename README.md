# image_undistort for ROS 2

## Installation

```bash
git clone https://github.com/kminoda/image_undistort_ros2.git
cd image_undistort_ros2
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Usage
```bash
ros2 launch image_undistort image_undistort.launch.xml
```
