<img src=https://dreamvu.com/wp-content/uploads/2020/07/logo_footer_trans-1-1.png alt="DreamVU">

# ros2_pal_camera_node

This unoffical package creates a ros2 node which publishes the panoramic images (both color and depth) and a point cloud from [the DreamVU PAL camera](https://dreamvu.com/pal-usb/).

An official ros1-package is available at [the DreamVU software page](https://dreamvu.com/software/), which is a package based on the Catkin build system.

This package here is based on the Colcon build system, which allows to build the package for the combination of Ubuntu 20.04 (Focal Fossa) with ROS2 Foxy Fitxroy.

This package lets you use the PAL camera with ROS2. It will provide access to the following data:

* Left and right unrectified images
* Depth data
* Colored 3D point-cloud

<img src="https://staff.fnwi.uva.nl/a.visser/research/roboresc/2021/robolab/left.jpg"
     alt="Left image"
     style="float: left; margin-right: 10px;" width=200/>
<img src="https://staff.fnwi.uva.nl/a.visser/research/roboresc/2021/robolab/right.jpg"
     alt="Right image"
     style="float: left; margin-right: 10px;" width=200/>
 <img src="https://staff.fnwi.uva.nl/a.visser/research/roboresc/2021/robolab/depth.jpg"
     alt="Depth image"
     style="float: left; margin-right: 10px;" width=200/><br>
 <img src="https://staff.fnwi.uva.nl/a.visser/research/roboresc/2021/robolab/rviz2_main_window.png"
     alt="Point Cloud"
     style="float: left; margin-right: 10px;" width=615/>
     
## Known issues

* The published messages require a chain of coordinate transformations from the sensor to the robot base to the map to allow rviz2 to visualize the images and point cloud.
* The software is currently tested on two computers, more tests are on the way. 
* The node works with SDK v1.2 for Ubuntu 18.04 (18 April 2021), while you expect for ros-foxy that the version for 20.04 (15 May 2021) should be the best choice.

## Installation

### Prerequisites

* [Ubuntu 20.04 (Focal Fossa)](https://releases.ubuntu.com/focal/)

Next to the default packages, also install

```bash
sudo apt install usbutils
sudo apt install v4l-utils

suda add-apt-repository ppa:deadsnakes/ppa
sudo apt update
sudo apt install python3.6-dev
sudo apt install python3-pip
```

* [ROS2 Foxy Fitxroy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

If the ros-foxy repository is not already on your package list, add it

```bash
sudo apt update && sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
```

This ros-node is using the following ros-packages:

```bash
$ sudo apt install ros-foxy-ros-base
$ sudo apt install ros-foxy-rviz2
$ sudo apt install ros-foxy-image-transport
$ sudo apt install ros-foxy-camera-info-manager
$ sudo apt install ros-foxy-image-view
$ sudo apt install ros-foxy-v4l2-camera
```
* [Colcon build system](https://docs.ros.org/en/foxy/Tutorials/Colcon-Tutorial.html)
```bash
$ sudo apt install python3-rosdep2
$ sudo apt install python3-colcon-common-extensions
```
* [PAL USB SDK](https://dreamvu.com/software/) - Tested with SDK v1.2 for Ubuntu 18.04 (24 April 2021)
```bash
$ unzip PAL-Firmware-v1.2-Intel-CPU.zip
$ cd PAL-Firmware-v1.2-Intel-CPU/installations/
$ cd camera_data
$ source setup_python_lib.sh
$ cd ..
$ sudo ./PAL_udev.sh
REBOOT
```
> After unzipping the SDK, remember this directory location as PAK_SDK_DIR for the script that has to be executed during the Build
* Calibration package for the DreamVU camera with your serial number (contact DreamVU support for this package, this package is especially made for your camera).

```bash
$ unzip PUM*.zip
$ cd PUM*/
$ source setup.sh
$ cd ..
```

*Note that not the complete installation procedure of the PAL USB SDK have to be followed, the three steps described above are enough for this ros2 package*.

## Build the package

To install the **ros2 pal_camera_node**, clone the package from github and build it:

```bash
$ cd ~/ros2_ws/src/ #use your current ros2 workspace folder
$ git clone https://github.com/physar/ros2_pal_camera_node.git
$ cd ./ros2_pal_camera_node/pal_camera/
$ source ./etc/dreamvu/logic_link.sh PAL_SDK_DIR
$ cd ../../..
$ sudo apt-get update
$ rosdep update
$ rosdep install --from-paths src --ignore-src -r -y
$ colcon build --packages-select pal_camera --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
$ source $(pwd)/install/local_setup.bash
```

If you want to install this package permanently ot your shell, 

```bash
$ echo source $(pwd)/install/local_setup.bash >> ~/.bashrc
$ source ~/.bashrc
```

# Starting the PAL camera node

To start the **ros2 pal_camera_node**, open a terminal and start it with the command.

```bash
$ source /opt/ros/foxy/setup.bash
$ source $~/ros2_ws/install/local_setup.bash
$ ros2 run pal_camera capture
```

# Inspect the published topics

The easiest way to inspect the published data is the following command:

```bash
$ ros2 topic list | grep dreamvu
```
This should give the following result:

```bash
/dreamvu/pal/get/camera_info
/dreamvu/pal/get/depth
/dreamvu/pal/get/left
/dreamvu/pal/get/point_cloud
/dreamvu/pal/get/right
```
The next way to see the panoramic images is with image_view:
```bash
$ ros2 run image_view image_view --ros-args --remap image:=/dreamvu/pal/get/right
```

The complete set of published data can be seen by the following command:
```bash
ros2 run rviz2 rviz2
```

Add in this view an Image (for the topics ```/dreamvu/pal/get/left```, ```/dreamvu/pal/get/right```, ```/dreamvu/pal/get/depth``` or a PointCloud2 (for topic ```/dreamvu/pal/get/point_cloud```).

*In later versions of this package a launch script with for rviz2 will be provided*.



# Troubleshooting

* If your PAL camera is not connected to a USB 3.1, it is not visible with <tt>lsusb | grep See3CAM</tt>. Use in that case a faster USB port.

* If you are using the 20.04 version of the USB, you will encounter an PAL_MIDAS::Init() assertion. Use the 18.04 version of the software, by running the logic_link script again.

* rviz2 is dropping messages. This means that your robot is not publishing any coordinate transformations from the map to the base_link. The camera images are published from the coordinate system 'pal_camera_center'.
