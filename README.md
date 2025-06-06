# iridescence_ros
iridescence_ros provides ROS integration for the [Iridescence](https://github.com/koide3/iridescence) visualization toolkit. It includes a lightweight GUI based on ImGui that allows you to view and publish ROS topics directly from an interactive window.

## Features

- **Viewer integration** – uses Iridescence's OpenGL viewer to display ROS data.
- **ROS Topic Manager** – browse available ROS topics and subscribe to image streams.
- **Pseudo Image Publisher** – publish an image from disk on a user-specified topic.
- **Robot control mode** – choose between manual and autonomous modes from the main panel.
- **Font Awesome and Material Symbols** – embedded icon fonts for a modern interface.

## Installation

These instructions assume ROS Noetic on Ubuntu. The project also depends on `iridescence`, PCL, OpenCV and other common libraries.

1. Clone the repository and its submodules:

```bash
git clone --recursive https://github.com/Seekerzero/iridescence_ros.git
cd iridescence_ros
```

2. Install dependencies (replace `$ROS_DISTRO` with your ROS version name):

```bash
sudo apt-get install ros-$ROS_DISTRO-roscpp ros-$ROS_DISTRO-rospy ros-$ROS_DISTRO-std-msgs ros-$ROS_DISTRO-cv-bridge libpcl-dev libeigen3-dev libjpeg-dev libpng-dev
```

Ensure the [Iridescence](https://github.com/koide3/iridescence) library is installed on your system so CMake can find it.


3. Build with catkin:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
ln -s /path/to/iridescence_ros .
cd ..
catkin_make
```

4. Source the workspace and run the viewer:

```bash
source devel/setup.bash
rosrun iridescence_ros_tools viewer
```

## License

This project is released under the MIT License. See [LICENSE](LICENSE) for details.

