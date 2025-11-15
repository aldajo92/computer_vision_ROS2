# Computer Vision ROS2 Package

This package provides computer vision capabilities for ROS2 Humble, including:
- C++ node for computer vision tasks (`cv_node`)
- Python image publisher node that publishes images from a dataset as a video stream

## Installation

```bash
# Install the ROS 2 Humble packages for computer vision
sudo apt update
sudo apt install ros-humble-cv-bridge ros-humble-opencv python3-opencv

# Build the package
cd ~/RoboticsProjects/ROS2_Docker_UI/ros2_ws
colcon build --packages-select computer_vision

# Source the workspace
source install/setup.bash
```

## Image Publisher Node

The `image_publisher_node.py` publishes images from the `dataset/data` folder as a video stream at a configurable frequency.

### Features
- Publishes enumerated images (e.g., 0000000000.png, 0000000001.png, etc.)
- Configurable maximum index
- Configurable publish frequency (Hz)
- Optional looping through images
- Publishes as `sensor_msgs/Image` messages

### Parameters

- `max_index` (int, default: 113): Maximum index of images to publish
- `publish_frequency` (float, default: 10.0): Frequency in Hz at which to publish images
- `loop` (bool, default: true): Whether to loop through images continuously
- `topic_name` (string, default: '/camera/image_raw'): Topic name for publishing images

### Usage

#### Run with default parameters:
```bash
ros2 run computer_vision image_publisher_node.py
```

#### Run with custom parameters:
```bash
ros2 run computer_vision image_publisher_node.py --ros-args \
  -p max_index:=50 \
  -p publish_frequency:=5.0 \
  -p loop:=false \
  -p topic_name:=/my_camera/image
```

#### Using the launch file:
```bash
# Default settings (10 Hz, loop enabled, max_index=113)
ros2 launch computer_vision image_publisher.launch.py

# Custom settings
ros2 launch computer_vision image_publisher.launch.py \
  max_index:=100 \
  publish_frequency:=30.0 \
  loop:=true \
  topic_name:=/camera/image_raw
```

### Viewing the Published Images

To visualize the published images:

```bash
# Using rqt_image_view
ros2 run rqt_image_view rqt_image_view

# Or using RViz2
rviz2
# Then add an Image display and set the topic to /camera/image_raw
```

### Inspecting the Topic

```bash
# List topics
ros2 topic list

# Echo topic info
ros2 topic info /camera/image_raw

# Display publishing rate
ros2 topic hz /camera/image_raw
```

## Dataset

The dataset folder contains enumerated PNG images (0000000000.png to 0000000113.png) that are published sequentially by the image publisher node.
