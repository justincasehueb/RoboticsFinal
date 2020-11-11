# RoboticsFinal
ROS tour robot - Takes you on an autonomous tour of Devon Energy Hall on the University of Oklahoma's campus.

## Dependencies:
- **Catkin**
- **ROS Kinetic**
- **Gazebo**
- **Python 2.7.x**

## How to use:
1. Create an empty directory.
> ~/Example/parent/empty
2. Execute `catkin_make` in the parent directory to generate the file structure. You should see the same folders and structure as in this repo.
> ~/Example/parent
3. *Clone* or simply download this repo; overwrite files in your empty directory and preserve the structure in this repo and in your new catkin workspace.
> ~/Example/parent/empty
4. Execute `catkin_make` in the parent directory once again, finalizing the changes you just made to the files.
> ~/Example/parent/
5. Execute `source ~/Example/devel/setup.bash` to allow ros to run the behaviors in the node.
6. Execute `roslaunch proj1 proj1.launch` to initialize the world with Turtlebot
7. Execute `rosrun proj1 project_node.py` to have Turtlebot execute the node.
8. Reference the map to choose a node (currently numbered 0-11) which you would like to visit.
9. You will start at node "0". Tell Turtlebot where you want to go by inputting the integer value of any node on the map. Turtlebot will plan and execute a path to that node autonomously, then prompt again for another node when it has arrived. Repeat until you find joy and/or inner peace.

![Map](https://github.com/justincasehueb/RoboticsFinal/blob/main/DEH_map.jpg?raw=true)