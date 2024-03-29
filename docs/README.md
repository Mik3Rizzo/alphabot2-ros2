# ROS2 for Waveshare Alphabot2-Pi

This repo contains **ROS2** packages for the Waveshare **Alphabot2-Pi** mobile robot:
- **alphabot2**: nodes used to manage Alphabot2 sensors and actuators.
- **alphabot2_interfaces**: custom interfaces (ROS messages) designed for AlphaBot2.

The current implementation includes nodes to manage the **motors**, the **IR obstacle sensors** and the **camera**. 
There is also a node for **QR-code** detection.

> Please, see [here](ros2-topology.md) for the ROS2 nodes topology and TODO.

The code has been originally developed for my final project of the Robotics course, University of Brescia.


# Requirements

- **Robot**: Waveshare Alphabot2-Pi mobile robot (see [here](https://www.waveshare.com/wiki/AlphaBot2-Pi)), equipped with a Raspberry Pi4 (8 GB).

- **OS**: Ubuntu Server 20.04 (Focal Fossa) x64.


## Ubuntu and ROS2 setup

Install Ubuntu Server 20.04 x64 on the RPi4 (see [here](https://ubuntu.com/download/raspberry-pi)).

Connect the RPi4 to WiFi and add auto-connection at boot (see [here](https://www.linuxbabe.com/ubuntu/connect-to-wi-fi-from-terminal-on-ubuntu-18-04-19-04-with-wpa-supplicant)).

Install ROS2 Foxy Fitzroy Base ([reference](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)) and 
the dependencies:
``` bash
sudo apt update && sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt install ros-foxy-ros-base
```

Install python dependencies:
``` bash
sudo apt install python3-pip python3-colcon-common-extensions python3-rosdep2
pip3 install RPi.GPIO argcomplete
```

Initialize rosdep and source the ROS2 core environment:
``` bash
rosdep update
source /opt/ros/foxy/setup.bash
```

Fix GPiO permission problem ([reference](https://github.com/gpiozero/gpiozero/issues/837)):
``` bash
sudo apt install rpi.gpio-common
sudo usermod -aG dialout ubuntu
```

### Suggestions

Source the ROS2 core environment (**underlay**) and the ROS2 local workspaces (**overlays**) in the file ```.bashrc``` so that
they are automatically sourced in every bash shell:
``` bash
source /opt/ros/foxy/setup.bash
source /path_to_your_ros_ws/install/local_setup.bash
```

Plug an additional external network adapter and setup it as a WiFi Access Point to easily SSH into the RPi4.
Please follow [this guide](https://gist.github.com/ExtremeGTX/ea1d1c12dde8261b263ab2fead983dc8) and also these additional tricks:
- put ```optional: true``` for wlan0 in 50-cloud-init.yaml to prevent endless booting time.
- don't put ```After=network-online.target``` and ```Wants=network-online.target```
- ```sudo systemctl enable dnsmasq```


## Camera node (v4l2_camera)

For the Raspberry Camera management we use the ROS2 implementation v4l2_camera ([reference](https://index.ros.org/r/v4l2_camera/)).

Install required tools:

``` bash
sudo apt install v4l-utils
echo bcm2835-v4l2 | sudo tee -a /etc/modules
```

Add ```start_x=1``` in /boot/config.txt to enable the camera module.

Clone the camera node and dependencies needed to compress images into the ```src``` folder of your ros workspace:
``` bash
git clone -b foxy https://gitlab.com/boldhearts/ros2_v4l2_camera.git
cd ros2_v4l2_camera ; git checkout 269573bd ; cd ..
git clone -b ros2 https://github.com/ros-perception/vision_opencv.git
cd vision_opencv ; git checkout 9ea89084 ; cd ..
git clone -b ros2 https://github.com/ros-perception/image_common.git
cd image_common ; git checkout 9729de81 ; cd ..             
git clone -b ros2 https://github.com/ros-perception/image_transport_plugins.git
cd image_transport_plugins ; git checkout 7ca90727 ; cd ..
```

To fix building errors, modify ```camera_info_manager.cpp:46``` (in ```image_common/camera_info_manager/```) to:
``` cpp
#include "rcpputils/get_env.hpp"
```

Source build:
``` bash
cd your_ros_ws
rosdep install --from-paths src -r -y
colcon build
```

### Troubleshooting

If you get a build error like this:
```
Imported target "Boost::python" includes non-existent path
   "/include"
in its INTERFACE_INCLUDE_DIRECTORIES.
```

The Boost library may be installed in /usr/include and not in /include.
You can fix the problem creating a symlink:
```bash
sudo ln -s /usr/include /include
```
If /usr/include is not your case, you should figure out where the Boost library is placed and create a symlink accordingly.


# Build and usage

After you have all the requirements satisfied, clone this repo in ```your_ros_ws/src``` and build it:
``` bash
git clone https://github.com/Mik3Rizzo/alphabot2-ros2
cd your_ros_ws
colcon build
```

I recommend to run all the nodes at once using the launch file:
``` bash
ros2 launch alphabot2 alphabot2_launch.py
```

To move the robot, publish a ```Twist``` ROS2 message on the ```alphabot2/cmd_vel``` topic. For example:
``` bash
ros2 topic pub --rate 1 alphabot2/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 5.0}}"
```

This command publishes every second a linear velocity of ```1 m/s``` and an angular rate of ```5 rad/s```.
> Please note that the maximum theoretical linear velocity is ```1.65 m/s``` while the angular rate is ```38.8 rad/s```.
These quantities are estimated with the robot suspended from the floor and supplied with two Sony US14500VR Li-ion 3.7V 
batteries at full charge. In these conditions the maximum wheel RPM is 750.

# About

**Michele Rizzo**, *Master's Degree Computer Engineering student at University of Brescia*.
- **Mail**: [m.rizzo006@studenti.unibs.it](mailto:m.rizzo006@studenti.unibs.it).
- **Github**: [Mik3Rizzo](https://github.com/Mik3Rizzo/)

Please, feel free to contact me for informations or problems.
