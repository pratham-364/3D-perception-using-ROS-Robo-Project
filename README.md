# 3D-perception-using-ROS-Robo-Project
> to check gazebo version
* gazebo --version
> creating an active ROS workspace
* mkdir -p ~/catkin_ws/src
* cd ~/catkin_ws/
* catkin_make
> Clone this repository into the workspace
* cd ~/catkin_ws/src
* git clone https://github.com/pratham-364/3D-perception-using-ROS-Robo-Project.git
> again go in the active workspace and install rosdep
* cd ~/catkin_ws
* rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
* cd ~/catkin_ws/src/3D-perception-using-ROS-Robo-Project/kuka_arm/scripts
* sudo chmod +x target_spawn.py
* sudo chmod +x IK_server.py
* sudo chmod +x safe_spawner.sh
> Build the Project
* cd ~/catkin_ws
* catkin_make
> Add following to your .bashrc file
* export GAZEBO_MODEL_PATH=~/catkin_ws/src/3D-perception-using-ROS-Robo-Project/kuka_arm/models

* source ~/catkin_ws/devel/setup.bash
> Fianlly launch the project by
* cd ~/catkin_ws/src/3D-perception-using-ROS-Robo-Project/kuka_arm/scripts
* ./safe_spawner.sh
> now when the RVIZ and that fucking GAZEBO is running, you can click on "next" or "continue" to make the robot move 
> The demo ends when the robot arm reaches at the top of the drop location.
> There is no loopback implemented yet, so you need to close all the terminal windows in order to restart.
> That's all Folks!
> bye and let's all give a big fuck to Abraham
