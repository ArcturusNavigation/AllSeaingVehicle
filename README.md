# AllSeaingVehicle: Coastal and Aquaculture Marine Autonomy
# Getting Started
## Prerequisites
Running vehicle nodes
- ROS Noetic
- ZED SDK  
Runing simulation:
- Gazebo: Install gazebo 9 at https://classic.gazebosim.org/download.

To download the ZED SDK, find and download the correct installer from https://www.stereolabs.com/developers/release/ or use curl
```
curl -L https://download.stereolabs.com/zedsdk/3.8/l4t32.6/jetsons -o installer.run # Replace link with https://download.stereolabs.com/zedsdk/3.8/cu117/ubuntu20 if using docker
chmod +x installer.run 
```
Run installer
```
sudo apt install zstd # Run this if on a linux/jetson OS
./installer.run
```
Clone the repository:
```
git clone https://github.com/ArcturusNavigation/AllSeaingVehicle.git
```
If you're on a jetson device, you'll also need to create an alias to the path for opencv
```
sudo ln -s /usr/include/opencv4/opencv2 /usr/include/opencv 
```
## Building repository
First, we need to install all the rospackage dependencies. Navigate to the top-level of the repository then run
```
rosdep install --from-paths src --ignore-src -r -y 
```
Next, we'll build the repository and source the build
```
catkin_make
source devel/setup.bash
```
## Testing 

# Tutorials
## Running the gazebo simulation

1. Start up the gazebo simulation
```
roslaunch sim_suite boat.launch
```

In the simulation, delete the boat_with_sensors model then insert another one(fixes weird glitch). Then you can hit play to start the simulation.

2. Run the SITL simulation from arduRover

```
sim_vehicle.py -v APMrover2 --console
```

3. Run mavros with 

```
roslaunch arcturus_pilot mavros
```

