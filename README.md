///////////////////////////////////////////////////////////////////////////////////////////
###Go to for ROBOHUB PANDA ARM
1. https://franka1.robohub.eng.uwaterloo.ca/
2. Open brakes, (unlock) robot moves a little.
3. Open hamburger menu -> activate FCI
###To rebuild /ws, you need to:
sudo apt-get install xorg-dev libglu1-mesa-dev (if you exit the docker)
catkin clean 
y
catkin build
###Jupyter Notebook
http://localhost:8888/tree
/////////////////////////////////////////////////////////////////////////////////////////

###STARTING DOCKER CONTAINER
###Terminal 1
m23qures@robotics:~/robohub/panda$ ./uw_panda/start.sh panda_saved_image
m23qures@docker:~$ 
robohub@docker:~$ cd ws_moveit/
robohub@docker:~$ export DISPLAY=:0
robohub@docker:~/ws$ source devel/setup.bash  #source devel/setup.bash
robohub@docker:~/ws$ roslaunch panda_moveit_config demo.launch rviz_tutorial:=true

###Removing Gripper 
roslaunch panda_moveit_config demo.launch rviz_tutorial:=true load_gripper:=false

###Terminal 2

robohub@docker:~$ cd catkin_ws/
robohub@docker:~$ export DISPLAY=:0
robohub@docker:~/ws$ source devel/setup.bash  #source devel/setup.bash
robohub@docker:~/ws$ roslaunch 

\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
###IF ws_moveit is crashed
source /opt/ros/noetic/setup.bash
catkin clean
export CMAKE_PREFIX_PATH=/opt/ros/noetic:$CMAKE_PREFIX_PATH
sudo apt update
sudo apt upgrade
rosdep update
rosdep install --from-paths src --ignore-src -r -y
//////////////////////////////////////////////////////////////////////////////////////////

