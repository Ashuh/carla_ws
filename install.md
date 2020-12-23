# Installation Instructions (Ubuntu 18.04 / Melodic)

## Build CARLA

---
Referenced from the [CARLA documentation](https://carla.readthedocs.io/en/latest/build_linux/)
<details>
<summary>Show instructions</summary>

### Install Dependencies

```bash
sudo apt-get update &&
sudo apt-get install wget software-properties-common &&
sudo add-apt-repository ppa:ubuntu-toolchain-r/test &&
wget -O - https://apt.llvm.org/llvm-snapshot.gpg.key|sudo apt-key add - &&
sudo apt-add-repository "deb http://apt.llvm.org/$(lsb_release -c --short)/ llvm-toolchain-$(lsb_release -c --short)-8 main" &&
sudo apt-get update
```

```bash
# Additional dependencies for Ubuntu 18.04
sudo apt-get install build-essential clang-8 lld-8 g++-7 cmake ninja-build libvulkan1 python python-pip python-dev python3-dev python3-pip libpng-dev libtiff5-dev libjpeg-dev tzdata sed curl unzip autoconf libtool rsync libxml2-dev &&
pip2 install --user setuptools &&
pip3 install --user -Iv setuptools==47.3.1
```

```bash
# Change default clang version
sudo update-alternatives --install /usr/bin/clang++ clang++ /usr/lib/llvm-8/bin/clang++ 180 &&
sudo update-alternatives --install /usr/bin/clang clang /usr/lib/llvm-8/bin/clang 180
```

### Install Unreal Engine

__1.__ Create an [Unreal Engine](https://www.unrealengine.com/en-US/feed) account to access the Unreal Engine repositories, which are set to private.

__2.__ Connect both your GitHub and Unreal Engine accounts. Go to your personal settings in there is a section in Unreal Engine's website. Click on Connections > Accounts, and link both accounts. [Here](https://www.unrealengine.com/en-US/blog/updated-authentication-process-for-connecting-epic-github-accounts) is a brief explanation just in case.

```bash
# Download Unreal Engine 4.24
git clone --depth=1 -b 4.24 https://github.com/EpicGames/UnrealEngine.git ~/UnrealEngine_4.24
cd ~/UnrealEngine_4.24

# Download and install the UE patch
wget https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/UE_Patch/430667-13636743-patch.txt 430667-13636743-patch.txt
patch --strip=4 < 430667-13636743-patch.txt

# Build UE
./Setup.sh && ./GenerateProjectFiles.sh && make

# Open the UE Editor to check everything works properly
cd ~/UnrealEngine_4.24/Engine/Binaries/Linux && ./UE4Editor

# Set the environment variable
echo 'export UE4_ROOT=~/UnrealEngine_4.24' >> ~/.bashrc

```

### Install CARLA

```bash
# Clone the CARLA repository
git clone https://github.com/carla-simulator/carla

# Get the CARLA assets
cd ~/carla
./Update.sh

# make the CARLA client and the CARLA server
make PythonAPI
make launch
make PythonAPI ARGS="--python-version=2"

# Enable CARLA API
echo 'export PYTHONPATH=$PYTHONPATH:/home/ashuh/carla/PythonAPI/carla/dist/carla-0.9.10-py2.7-linux-x86_64.egg' >> ~/.bashrc

```

</details>
</br>

## Install Autoware Dependencies

---
Referenced from the [Autoware documentation](https://github.com/Autoware-AI/autoware.ai/wiki/Source-Build)

<details>
<summary>Show instructions</summary>

### Install System Dependencies

```bash
sudo apt update
sudo apt install -y python-catkin-pkg python-rosdep ros-$ROS_DISTRO-catkin
sudo apt install -y python3-pip python3-colcon-common-extensions python3-setuptools python3-vcstool
pip3 install -U setuptools
```

### Install CUDA 10.0

```bash
TODO
```

### Update Eigen

```bash
cd && wget http://bitbucket.org/eigen/eigen/get/3.3.7.tar.gz #Download Eigen
mkdir eigen && tar --strip-components=1 -xzvf 3.3.7.tar.gz -C eigen #Decompress
cd eigen && mkdir build && cd build && cmake .. && make && make install #Build and install
cd && rm -rf 3.3.7.tar.gz && rm -rf eigen #Remove downloaded and temporary files
```

</details>
</br>

## Install Other Dependencies

---

```bash
# Install pygame for manual control
pip install pygame
```

```bash
# Install ROS packages for localization
sudo apt-get install ros-$ROS_DISTRO-map-server &&
sudo apt-get install ros-$ROS_DISTRO-slam-gmapping &&
sudo apt-get install ros-$ROS_DISTRO-pointcloud-to-laserscan &&
sudo apt-get install ros-$ROS_DISTRO-robot-localization &&
sudo apt-get install ros-$ROS_DISTRO-amcl
```

## Install Workspace

---

```bash
git clone https://github.com/Ashuh/carla_ws.git
cd carla_ws
git submodule update --init --recursive
rosdep install -y --from-paths src --ignore-src -r
catkin_make
```

## Download Maps

---

```bash
cd "$HOME"/carla_ws/src/carla_agv/agv/autoware-contents
git lfs pull
```
