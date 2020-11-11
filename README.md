# RoboticsFinal
ROS tour robot - Takes you on an autonomous tour of Devon Energy Hall on the University of Oklahoma's campus.

## Dependencies:
- **Catkin**
- **ROS Kinetic**
- **Gazebo**
- **Python 2.7.x**

## How to use:
1. *Clone* this repo into an empty directory. 
> ~/Example/parent/empty
2. Execute `catkin_make` in the parent directory. 
> ~/Example/parent/
3. Execute `source ~/Example/devel/setup.bash` to allow ros to run the behaviors in the node.
4. Execute `roslaunch proj1 proj1.launch` to initialize the world with Turtlebot
5. Execute `rosrun proj1 project_node.py` to have Turtlebot execute the node.
6. Reference the map to choose a node (currently numbered 0-11) which you would like to visit.
7. You will start at node "0". Tell Turtlebot where you want to go by inputting the integer value of any node on the map. Turtlebot will plan and execute a path to that node autonomously, then prompt again for another node when it has arrived. Repeat until you find joy and/or inner peace.
