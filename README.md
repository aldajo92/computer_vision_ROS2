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

## CV Processing Node (C++)

The `cv_node` subscribes to images, processes them with OpenCV, and publishes the results.

### Topics

- **Subscribes to:** `/camera/image_raw` (sensor_msgs/Image)
- **Publishes to:** `/cv/gray_image` (sensor_msgs/Image)

### Processing

Currently performs grayscale conversion. You can modify the processing in `src/cv_node.cpp` to add your own computer vision algorithms.

### Usage

```bash
# Run the CV processing node standalone
ros2 run computer_vision cv_node

# View the processed images
ros2 run rqt_image_view rqt_image_view
# Select /cv/gray_image from the dropdown
```

## Complete CV Pipeline Launch

The `cv_node_process.launch.py` launch file runs both the image publisher and the C++ subscriber/processor together:

```bash
# Run both publisher and subscriber
ros2 launch computer_vision cv_node_process.launch.py

# With custom parameters for the publisher
ros2 launch computer_vision cv_node_process.launch.py \
  max_index:=100 \
  publish_frequency:=15.0
```

This launch file:
- **image_publisher_node.py**: Publishes images from the dataset at the specified frequency to `/camera/image_raw`
- **cv_node**: Subscribes to `/camera/image_raw`, processes images (grayscale conversion), and publishes results to `/cv/gray_image`

### Visualizing Both Original and Processed Images

```bash
# Terminal 1: Run the complete pipeline
ros2 launch computer_vision cv_node_process.launch.py

# Terminal 2: View original images
ros2 run rqt_image_view rqt_image_view /camera/image_raw

# Terminal 3: View processed images
ros2 run rqt_image_view rqt_image_view /cv/gray_image
```

### Available Topics

When running the complete pipeline, you'll have:
- `/camera/image_raw` - Original color images from dataset
- `/cv/gray_image` - Processed grayscale images

## Dataset

The dataset folder contains enumerated PNG images (0000000000.png to 0000000113.png) that are published sequentially by the image publisher node.
