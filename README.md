# CARLA Workspace

## Install CARLA

```bash
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 1AF1527DE64CB8D9
sudo add-apt-repository "deb [arch=amd64] http://dist.carla.org/carla bionic main"
sudo apt-get update # Update the Debian package index
sudo apt-get install carla-simulator # Install the latest CARLA version, or update the current installation
cd /opt/carla-simulator # Verify the CARLA installation
```

## Install ROS Packages

```bash
sudo apt-get install ros-$ROS_DISTRO-map-server
sudo apt-get install ros-$ROS_DISTRO-slam-gmapping
sudo apt-get install ros-$ROS_DISTRO-pointcloud-to-laserscan
sudo apt-get install ros-$ROS_DISTRO-robot-localization
sudo apt-get install ros-$ROS_DISTRO-amcl
```

## Install Workspace

```bash
git clone https://github.com/Ashuh/carla_ws.git
cd carla_ws
git submodule update --init --recursive
rosdep install --from-paths src --ignore-src -r
catkin_make
```

## Mapping

```bash
./mapping.bash
```

To save the map generated:

```bash
rosrun map_server map_saver [--occ <threshold_occupied>] [--free <threshold_free>] [-f <mapname>] map:=/your/costmap/topic
```
