# RoboHub Panda: Lab Setup Notes

Notes for running RAPTAR on the University of Waterloo RoboHub Franka Emika Panda through the lab's ROS Noetic Docker image.

## 1. Prepare the robot

1. Open the Franka Desk interface: https://franka1.robohub.eng.uwaterloo.ca/
2. Open the brakes to unlock the robot. It moves slightly when they release.
3. Open the hamburger menu and choose **Activate FCI**.

## 2. Start the Docker container

Install MoveIt by following the [MoveIt tutorials](https://moveit.github.io/moveit_tutorials/) into a workspace named `ws_moveit`. Then use the lab's `uw_panda` scripts to start the ROS Noetic container:

```bash
./uw_panda/start.sh panda_saved_image
```

## 3. Terminal 1: MoveIt

```bash
cd ~/ws_moveit
export DISPLAY=:0
source devel/setup.bash
roslaunch panda_moveit_config demo.launch rviz_tutorial:=true
# without the gripper:
# roslaunch panda_moveit_config demo.launch rviz_tutorial:=true load_gripper:=false
```

## 4. Terminal 2: RAPTAR scripts

```bash
cd ~/catkin_ws
export DISPLAY=:0
source devel/setup.bash
rosrun panda_moveit_demo add_table.py
rosrun panda_moveit_demo attach_rectangle.py
rosrun panda_moveit_demo panda_motion_plan.py
```

A Jupyter server, if you run one in the container, is at http://localhost:8888/tree.

## Rebuilding the workspace

If you have exited the container, install the X11/GL headers first:

```bash
sudo apt-get install xorg-dev libglu1-mesa-dev
catkin clean
catkin build
```

## Recovering a broken `ws_moveit`

```bash
source /opt/ros/noetic/setup.bash
catkin clean
export CMAKE_PREFIX_PATH=/opt/ros/noetic:$CMAKE_PREFIX_PATH
sudo apt update && sudo apt upgrade
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```
