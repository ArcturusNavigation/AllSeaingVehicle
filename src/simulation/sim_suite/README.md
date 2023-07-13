# Prerequisites
## Ardupilot SITL Setup for Ubuntu
First, clone the ardupilot repository
```
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive
```
Install some required packages
```
Tools/environment_install/install-prereqs-ubuntu.sh -y
```
If you run into an error try running the following commands first
```
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin
sudo mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600
sudo apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/3bf863cc.pub
sudo add-apt-repository "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/ /"
```
Reload the path (log-out and log-in to make permanent):
```
. ~/.profile
```

Finally, we build the repository
```
./waf configure --board sitl --debug
./waf rover
```
If rebuilding after updates to submodule, make sure to clean
```
./waf clean
```
# Tutorials
## Displaying URDF
First, make sure you have urdf tutorial installed:
```
sudo apt-get install ros-noetic-urdf-tutorial
```
Then you can display the URDF in RVIZ using the display launch file. Replace model_path with the path to the urdf to be displayed
```
roslaunch urdf_tutorial display.launch model:=${model_path}
```
## Generating SDF from URDF/Xacro
1. (Optional if you already have a plain URDF) Convert xacro file to pure urdf 
```
xacro athena.urdf.xacro > athena.urdf
```
2. Convert URDF into SDF file 
```
gz sdf -p athena.urdf > athena.sdf
```
## Running Simulations
First, launch ardupilot sitl
```
 ~/ardupilot/Tools/autotest/sim_vehicle.py -v APMrover2 -f gazebo-rover --udp --console
```
Next, launch your gazebo world. For example, to launch the full competition map with the boat.
```
roslaunch sim_suite boat.launch verbose:=true 
```
Finally, launch whatever pilot nodes you are using. In our case, we also need to launch the sensor processing nodes.
```
roslaunch perception_suite perception.launch 
```

```
roslaunch sim_suite gazebo_pilot.launch
```
