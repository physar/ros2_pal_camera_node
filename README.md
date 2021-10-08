# ros2_pal_camera_node

This package creates a ros2 node which publishes the panoramic images (both color and depth) and a point cloud from [the DreamVU PAL camera](https://dreamvu.com/pal-usb/).

An official ros1-package is available at [the DreamVU software page](https://dreamvu.com/software/), which is a package based on the Catkin build system.

This package is based on the Colcon build system, which allows to build the package for the combination of Ubuntu 20.04 (Focal Fossa) with ROS2 Foxy Fitxroy.

This package lets you use the PAL camera with ROS2. It will provide access to the following data:

* Left and right unrectified images
* Depth data
* Colored 3D point-cloud

## Known issues

The published 3D point-cloud seems to requires a computer with a GPU, to allow rviz2 to visualize the point cloud.

## Installation

### Prerequisites

* [Ubuntu 20.04 (Focal Fossa)](https://releases.ubuntu.com/focal/)
* [ROS2 Foxy Fitxroy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
** <code>sudo apt install ros-foxy-ros-base</code>
** <code>sudo apt install ros-foxy-rviz2</code>
** <code>sudo apt install ros-foxy-image-transport</code>
** <code>sudo apt install ros-foxy-camera-info-manager </code>
** <code>sudo apt install ros-foxy-image-view</code>
* [Colcon build system](https://docs.ros.org/en/foxy/Tutorials/Colcon-Tutorial.html)
** <code>sudo apt install python3-rosdep2</code>
** <code>sudo apt install python3-colcon-common-extensions</code>
* [PAL USB SDK](https://dreamvu.com/software/) - Tested with v1.2 for Ubuntu 20.04 (15 May 2021)
* Calibration package for the DreamVU camera with your serial number (contact DreamVU support for this package).

