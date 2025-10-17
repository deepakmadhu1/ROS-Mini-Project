#Examination Task
This ROS2 package is done for Mobile Robotics Examination Task.

Included in the package are:
* Package.xml and CMakeLists.txt files
* a urdf for a differential drive robot. Not including any external meshes, but with all required tags to work properly in Gazebo.
* a world file for gazebo with moveable walls.
* a config file for the differential drive controller
* a launch file for Gazebo and Rviz
* a config file for Rviz
* Map Folder with a Map of the World
* Meshes Folder Contains all the required 3D models for the task.
* Script Contain the Yellow Object finding Pythonn Code. 

#Note.
#All required packages in the practice pdf is preinstalled to the PC.

#Changes done in the parameter section of explorelite.
Explore Lite  min_frontier_size: 0.6 in params.yaml

#ADD all this to .bashrc for Before running the Project.

source /opt/ros/humble/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws/install/local_setup.bash
export GAZEBO_MODEL_PATH=~/ros2_ws/src/examtask/meshes
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

#For this Experiment we can used the explore lite for exploring the world
         git clone https://github.com/robo-friends/m-explore-ros2.git


#For running the python script we have installed the folllowing packages.
          sudo apt install python3-rosdep python3-colcon-common-extensions python3-argcomplete         
          sudo apt install ros-$ROS_DISTRO-cv-bridge
          sudo apt install python3-opencv python3-numpy
          sudo rosdep init
          sudo apt install ros-humble-tf2-ros ros-humble-tf2-geometry-msgs

#For Runing the Shell File install the gnome-terminal Package          
          sudo apt install gnome-terminal

#All the run commands can be seen in the examscript.sh file

#Load all the REQUIRED STL AND 3D Files for proper Working.




