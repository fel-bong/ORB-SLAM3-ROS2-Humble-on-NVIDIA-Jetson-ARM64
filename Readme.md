# ORB-SLAM3 ROS2 Humble on NVIDIA Jetson (ARM64)

This guide documents the complete installation process for ORB-SLAM3 with ROS2 Humble on NVIDIA Jetson platforms (tested on Jetson Orin with JetPack 6.2).

This is a fork/extension of [Mechazo11/ros2_orb_slam3](https://github.com/Mechazo11/ros2_orb_slam3) with Jetson-specific modifications.

## Overview

The original `ros2_orb_slam3` wrapper includes pre-built x86_64 libraries that won't work on ARM64 Jetson devices. This guide covers:

1. Building Pangolin from source with Jetson fixes
2. Building ORB-SLAM3 from source with C++14 compatibility
3. Configuring the ROS2 wrapper with ARM64 libraries
4. Setting up live camera input

## Tested Configuration

| Component | Version |
|-----------|---------|
| Platform | NVIDIA Jetson Orin |
| JetPack | 6.2.1 |
| Ubuntu | 22.04 |
| ROS2 | Humble Hawksbill |
| OpenCV | 4.5.4 |
| Eigen | 3.4.0 |

## Prerequisites

### Verify Existing Packages

```bash
# Check OpenCV version (4.2+ required)
python3 -c "import cv2; print(cv2.__version__)"

# Check Eigen version (3.1+ required, 3.4 recommended)
pkg-config --modversion eigen3
```

### Install Dependencies

```bash
sudo apt update
sudo apt install -y \
    libglew-dev \
    libboost-all-dev \
    libssl-dev \
    libepoxy-dev \
    ros-humble-vision-opencv \
    ros-humble-message-filters \
    ros-humble-cv-bridge
```

## Step 1: Build Pangolin

Pangolin is used for visualization. The standard build requires modifications for Jetson.

```bash
cd ~
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
```

### Jetson Fix: Remove -Werror Flag

The `-Werror` flag causes build failures on ARM64. **Important:** Use precise sed commands to avoid breaking other flags:

```bash
# Remove standalone -Werror only (preserve -Wno-error=...)
sed -i 's/ -Werror / /g' CMakeLists.txt
sed -i 's/ -Werror"/ "/g' CMakeLists.txt
sed -i 's/"-Werror /"/g' CMakeLists.txt
```

### Build and Install

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

### Verify Installation

```bash
ls /usr/local/include/pangolin/pangolin.h
ls /usr/local/lib/libpango_core.so
```

Both files should exist. Note: Pangolin uses `libpango_*` naming, not `libpangolin*`.

## Step 2: Build ORB-SLAM3

```bash
cd ~
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git
cd ORB_SLAM3
chmod +x build.sh
```

### Fix: C++14 Compatibility

Pangolin's sigslot header requires C++14, but ORB-SLAM3 defaults to C++11:

```bash
sed -i 's/++11/++14/g' CMakeLists.txt
```

Verify the change:

```bash
grep -n "CMAKE_CXX_STANDARD\|c++1" CMakeLists.txt
```

### Build

**Memory Warning:** Jetson devices may run out of memory with parallel builds. Use limited jobs:

```bash
cd build
rm -rf *
cmake ..
make -j2  # Use -j1 if -j2 still causes OOM kills
```

If you see `Killed signal terminated program cc1plus`, reduce parallelism further.

Build takes 15-30 minutes depending on your device.

### Install Sophus System-Wide

The ROS2 wrapper needs Sophus headers:

```bash
cd ~/ORB_SLAM3/Thirdparty/Sophus/build
sudo make install
```

## Step 3: Set Up ROS2 Wrapper

### Clone the Wrapper

```bash
mkdir -p ~/ros2_test/src
cd ~/ros2_test/src
git clone https://github.com/Mechazo11/ros2_orb_slam3.git
```

### Replace x86 Libraries with ARM64

The wrapper includes pre-built x86_64 libraries. Replace them with our ARM64 builds:

```bash
# Backup original Thirdparty
mv ~/ros2_test/src/ros2_orb_slam3/orb_slam3/Thirdparty \
   ~/ros2_test/src/ros2_orb_slam3/orb_slam3/Thirdparty.bak

# Copy ARM64 Thirdparty from our build
cp -r ~/ORB_SLAM3/Thirdparty ~/ros2_test/src/ros2_orb_slam3/orb_slam3/
```

### Fix: Vocabulary Loading

The wrapper uses binary vocabulary loading, but our DBoW2 build uses text loading:

```bash
# Change loadFromBinFile to loadFromTextFile
sed -i 's/loadFromBinFile/loadFromTextFile/g' \
    ~/ros2_test/src/ros2_orb_slam3/orb_slam3/src/System.cc

# Change vocabulary filename
sed -i 's/ORBvoc.txt.bin/ORBvoc.txt/g' \
    ~/ros2_test/src/ros2_orb_slam3/src/common.cpp

# Copy text vocabulary
cp ~/ORB_SLAM3/Vocabulary/ORBvoc.txt \
   ~/ros2_test/src/ros2_orb_slam3/orb_slam3/Vocabulary/
```

(UPLOADED IN THIS REPO AS ZIP, FILE WAS TOO LARGE, PLEASE UNZIP 'ORBvoc.txt.zip' FOR USE)

### Build the Wrapper

```bash
cd ~/ros2_test
source /opt/ros/humble/setup.bash
rosdep install -r --from-paths src --ignore-src -y --rosdistro humble

# Build with limited parallelism to avoid OOM
MAKEFLAGS="-j2" colcon build --symlink-install --packages-select ros2_orb_slam3
```

### Set Library Path

Pangolin libraries are in `/usr/local/lib`. Make them findable:

```bash
# Temporary (current session)
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

# Permanent (add to .bashrc)
echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
```

## Step 4: Test with Sample Dataset

### Terminal 1: ORB-SLAM3 Node

```bash
source ~/ros2_test/install/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
ros2 run ros2_orb_slam3 mono_node_cpp --ros-args -p node_name_arg:=mono_slam_cpp
```

### Terminal 2: Dataset Driver

```bash
source ~/ros2_test/install/setup.bash
ros2 run ros2_orb_slam3 mono_driver_node.py --ros-args \
    -p settings_name:=EuRoC \
    -p image_seq:=sample_euroc_MH05
```

You should see Pangolin windows open with the SLAM visualization.

## Step 5: Configure Your Camera

### Camera Calibration

If you haven't calibrated your camera:

```bash
# Install calibration package
sudo apt install ros-humble-camera-calibration

# Run calibrator (adjust size/square for your checkerboard)
ros2 run camera_calibration cameracalibrator \
    --size 9x6 \
    --square 0.025 \
    --ros-args -r image:=/camera/image/raw
```

Move the checkerboard around until all bars are green, then click CALIBRATE and SAVE.

### Create Camera Config

Create a YAML config file for your camera. Example for IMX219 at 960x540:

```yaml
%YAML:1.0

File.version: "1.0"
Camera.type: "PinHole"

# Camera intrinsics (from calibration)
Camera1.fx: 755.01546
Camera1.fy: 566.28059
Camera1.cx: 494.26509
Camera1.cy: 308.36827

# Distortion coefficients (k1, k2, p1, p2)
Camera1.k1: -0.368697
Camera1.k2: 0.252757
Camera1.p1: -0.007908
Camera1.p2: 0.007684

# Image dimensions
Camera.width: 960
Camera.height: 540
Camera.newWidth: 960
Camera.newHeight: 540

# Camera FPS
Camera.fps: 21

# Color order (0: BGR, 1: RGB)
Camera.RGB: 0

# ORB Parameters
ORBextractor.nFeatures: 1000
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7

# Viewer Parameters
Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1.0
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2.0
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3.0
Viewer.ViewpointX: 0.0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -1.8
Viewer.ViewpointF: 500.0
```

Save as `~/ros2_test/src/ros2_orb_slam3/orb_slam3/config/Monocular/YOUR_CAMERA.yaml`

## Step 6: Live Camera Driver

Create a driver node to bridge your ROS2 camera topic to ORB-SLAM3.

### Create the Driver

Save the following as `~/ros2_test/src/ros2_orb_slam3/scripts/live_mono_driver.py`:

```python
#!/usr/bin/env python3
"""
Live camera driver for ORB-SLAM3 monocular mode.
Bridges your camera topic to ORB-SLAM3's expected topics.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float64
from cv_bridge import CvBridge


class LiveMonoDriver(Node):
    def __init__(self):
        super().__init__('live_mono_driver')
        
        self.declare_parameter('settings_name', 'IMX219')
        self.declare_parameter('camera_topic', '/camera/image/raw')
        
        self.settings_name = self.get_parameter('settings_name').value
        self.camera_topic = self.get_parameter('camera_topic').value
        
        self.get_logger().info(f'Settings: {self.settings_name}')
        self.get_logger().info(f'Camera topic: {self.camera_topic}')
        
        self.br = CvBridge()
        self.send_config = True
        self.frame_count = 0
        
        # Publishers for ORB-SLAM3
        self.pub_config = self.create_publisher(
            String, '/mono_py_driver/experiment_settings', 1)
        self.pub_img = self.create_publisher(
            Image, '/mono_py_driver/img_msg', 1)
        self.pub_timestep = self.create_publisher(
            Float64, '/mono_py_driver/timestep_msg', 1)
        
        # Subscriber for ACK
        self.sub_ack = self.create_subscription(
            String, '/mono_py_driver/exp_settings_ack',
            self.ack_callback, 10)
        
        self.sub_camera = None
        
        # Match your camera's QoS
        self.camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.handshake_timer = self.create_timer(0.1, self.handshake_callback)
        self.get_logger().info('LiveMonoDriver initialized, attempting handshake...')
    
    def ack_callback(self, msg):
        if msg.data == 'ACK' and self.send_config:
            self.get_logger().info('Handshake complete!')
            self.send_config = False
            self.handshake_timer.cancel()
            
            self.sub_camera = self.create_subscription(
                Image, self.camera_topic,
                self.camera_callback, self.camera_qos)
            self.get_logger().info(f'Subscribed to {self.camera_topic}')
    
    def handshake_callback(self):
        if self.send_config:
            msg = String()
            msg.data = self.settings_name
            self.pub_config.publish(msg)
    
    def camera_callback(self, msg):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        timestep_msg = Float64()
        timestep_msg.data = timestamp
        self.pub_timestep.publish(timestep_msg)
        self.pub_img.publish(msg)
        
        self.frame_count += 1
        if self.frame_count % 100 == 0:
            self.get_logger().info(f'Published {self.frame_count} frames')


def main(args=None):
    rclpy.init(args=args)
    node = LiveMonoDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Register the Driver

Add to CMakeLists.txt in the install section:

```cmake
install(PROGRAMS
  scripts/mono_driver_node.py
  scripts/live_mono_driver.py  # Add this line
  DESTINATION lib/${PROJECT_NAME}
)
```

Make executable and rebuild:

```bash
chmod +x ~/ros2_test/src/ros2_orb_slam3/scripts/live_mono_driver.py
cd ~/ros2_test
MAKEFLAGS="-j2" colcon build --symlink-install --packages-select ros2_orb_slam3
```

## Step 7: Run with Live Camera

### Terminal 1: Camera Node

```bash
# Start your camera node (example)
ros2 run your_camera_pkg camera_node
```

### Terminal 2: Verify Camera

```bash
ros2 topic hz /camera/image/raw
```

### Terminal 3: ORB-SLAM3

```bash
source ~/ros2_test/install/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
ros2 run ros2_orb_slam3 mono_node_cpp --ros-args -p node_name_arg:=mono_slam_cpp
```

### Terminal 4: Live Driver

```bash
source ~/ros2_test/install/setup.bash
ros2 run ros2_orb_slam3 live_mono_driver.py --ros-args \
    -p settings_name:=YOUR_CAMERA \
    -p camera_topic:=/camera/image/raw
```

## Troubleshooting

### Library Not Found: libpango_display.so

```bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
sudo ldconfig
```

### Out of Memory During Build

Use single-threaded compilation:

```bash
make -j1
# or for colcon:
MAKEFLAGS="-j1" colcon build --symlink-install --packages-select ros2_orb_slam3
```

### QoS Incompatibility Warning

If you see "requesting incompatible QoS", ensure your camera subscriber uses matching QoS:

```python
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,  # Match publisher
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE
)
```

### Tracking Failures / Map Resets

Monocular SLAM is sensitive. For best results:

- Point camera at textured scenes (not blank walls)
- Move slowly and smoothly during initialization
- Use translational motion (not pure rotation)
- Ensure consistent lighting
- Once "New Map created with X points" appears, continue slow movement

### Black Screen / Viewer Freeze (SSH)

Sometimes i got a black screen when running the viewer, just restart it and it should work or disable the viewer:
(it can take a while to come up)

```yaml
# Add to your camera config YAML:
System.UseViewer: 0
```

### Vocabulary Loading Slow

The text vocabulary (~140MB) takes 1-2 minutes to load. This is normal for `loadFromTextFile`. The binary version is faster but requires matching DBoW2 versions.

## Summary of Jetson-Specific Changes

| Component | Change | Reason |
|-----------|--------|--------|
| Pangolin | Remove `-Werror` flag | ARM64 compiler warnings treated as errors |
| ORB-SLAM3 | C++11 → C++14 | Pangolin sigslot requires C++14 |
| ORB-SLAM3 | Build with `-j2` | Prevent OOM on limited RAM |
| ROS2 Wrapper | Replace Thirdparty libs | Pre-built x86_64 → ARM64 |
| ROS2 Wrapper | Binary → Text vocabulary | DBoW2 version compatibility |

## References

- [Original ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- [Mechazo11/ros2_orb_slam3](https://github.com/Mechazo11/ros2_orb_slam3)
- [Pangolin](https://github.com/stevenlovegrove/Pangolin)

## License

This guide follows the same license as the original repositories (GPLv3 for ORB-SLAM3).
