1. GOTO https://moveit.github.io/moveit_tutorials/ to install the MoveIt Setup as **ws_moveit**.
2. For Docker setup used uw_panda dir to run the docker container with ROS_Noetic setup.
# Hardware Setup with ROS: Go to for ROBOHUB PANDA ARM
1. https://franka1.robohub.eng.uwaterloo.ca/
2. Open brakes, (unlock) robot moves a little.
3. Open hamburger menu -> activate FCI
###To rebuild /ws, you need to:
sudo apt-get install xorg-dev libglu1-mesa-dev (if you exit the docker)
catkin clean 
catkin build
# Jupyter Notebook
1. http://localhost:8888/tree

# STARTING DOCKER CONTAINER
Terminal 1
1. m23qures@robotics:~/robohub/panda$ ./uw_panda/start.sh panda_saved_image
2. m23qures@docker:~$ 
3. robohub@docker:~$ cd ws_moveit/
4. robohub@docker:~$ export DISPLAY=:0
5. robohub@docker:~/ws$ source devel/setup.bash  #source devel/setup.bash
6. robohub@docker:~/ws$ roslaunch panda_moveit_config demo.launch rviz_tutorial:=true

# Removing Gripper 
1. roslaunch panda_moveit_config demo.launch rviz_tutorial:=true load_gripper:=false

###Terminal 2

1. robohub@docker:~$ cd catkin_ws/
2. robohub@docker:~$ export DISPLAY=:0
3. robohub@docker:~/ws$ source devel/setup.bash  #source devel/setup.bash
4. robohub@docker:~/ws$ roslaunch 

# IF ws_moveit is crashed
1. source /opt/ros/noetic/setup.bash
2. catkin clean
3. export CMAKE_PREFIX_PATH=/opt/ros/noetic:$CMAKE_PREFIX_PATH
4. sudo apt update
5. sudo apt upgrade
6. rosdep update
7. rosdep install --from-paths src --ignore-src -r -y

