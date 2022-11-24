# AllSeaingVehicle: Coastal and Aquaculture Marine Autonomy
# Getting Started
## Prerequisites
Running vehicle nodes
- ROS Noetic
- ZED SDK
Runing simulation:
- Gazebo

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
