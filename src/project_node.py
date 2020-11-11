#! /usr/bin/env python

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations

import math

graph = {'0': ['1','10'],
         '1': ['2','3','0'],
         '2': ['1'],
         '3': ['1','4'],
         '4': ['3', '5'],
         '5': ['4','6'],
         '6': ['5','7'],
         '7': ['6','8','9'],	
         '8': ['7'],
         '9': ['10','7'],
         '10': ['11','0','9'],
         '11': ['10'] }

nodeCoords = [[0,0],[-1.5,-1.5],[0,-3],[-4,-4],[-8.5,1],[-4,5.5],[1,10],[4,7],[6,9],[6,5],[3,2],[5,0]]

pathList=['0']

currentNode=0

# robot state variables
position_ = Point()
yaw_ = 0
# machine state
state_ = 2
# goal
desired_position_ = Point()
desired_position_.x = 0
desired_position_.y = 0
desired_position_.z = 0
# parameters
yaw_precision_ = (math.pi / 90) # +/- 2 degree allowed
dist_precision_ = 0.1

# publishers
pub = None

def fix_yaw(des_pos):
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_

    twist_msg = Twist()

    if math.fabs(err_yaw) > yaw_precision_:
	#print 'turning...'
        twist_msg.angular.z = 0.3 if err_yaw > 0 else -0.3

    pub.publish(twist_msg)
    
    # state change conditions
    #print 'Yaw error: [%s]' % err_yaw
    #print 'Yaw prec: [%s]' % yaw_precision_
    if math.fabs(err_yaw) <= yaw_precision_:
        #print 'Yaw error: [%s]' % err_yaw
        change_state(1)

def go_straight_ahead(des_pos):
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))
    
    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 0.3
        pub.publish(twist_msg)
    else:
        #print 'Position error: [%s]' % err_pos
        change_state(2)
    
    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        print 'Yaw error: [%s]' % err_yaw
        change_state(0)

def done():
    global twist_msg
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)

def change_state(state):
    global state_
    state_ = state
    print 'State changed to [%s]' % state_

def clbk_odom(msg):
    global position_
    global yaw_
    
    # position
    position_ = msg.pose.pose.position
    
    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

def bfs(start, goal):
    global graph
    # keep track of explored nodes
    explored = []
    # keep track of all the paths to be checked
    queue = [[start]]
 
    # return path if start is goal
    if start == goal:
        return [start]
 
    # keeps looping until all possible paths have been checked
    while queue:
        # pop the first path from the queue
        path = queue.pop(0)
        # get the last node from the path
        node = path[-1]
        if node not in explored:
            neighbours = graph[node]
            # go through all neighbour nodes, construct a new path and
            # push it into the queue
            for neighbour in neighbours:
                new_path = list(path)
                new_path.append(neighbour)
                queue.append(new_path)
                # return path if neighbour is goal
                if neighbour == goal:
                    return new_path
 
            # mark node as explored
            explored.append(node)
 
    # in case there's no path between the 2 nodes
    return [-1]

def main():
    global pub,pathList,currentNode,graph
    
    print 'Initialized....:)'

    rospy.init_node('go_to_point')
    
    pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
    
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if state_ == 0:
            #print 'state 0'
            fix_yaw(desired_position_)
        elif state_ == 1:
            #print 'state 1'
            go_straight_ahead(desired_position_)
        elif state_ == 2:
            #print 'state 2'
            done()
	    currentNode=pathList[0]
	    pathList.pop(0)
	    if len(pathList)==0:
		print 'Enter destination node:'
		dest=int(raw_input())
		pathList=bfs(str(currentNode),str(dest))
		print(pathList)
	    desired_position_.x=nodeCoords[int(pathList[0])][0]
	    desired_position_.y=nodeCoords[int(pathList[0])][1]
#	    print 'Enter node to travel to:'
#	    userDest=int(raw_input())
#	    print 'Travelling to node %s' % userDest
#	    
#	    if userDest>=0:
#	    	desired_position_.x=nodeCoords[userDest][0]
#	    	desired_position_.y=nodeCoords[userDest][1]
#	    else:
#	    	print 'enter x coordinate:'
#	    	desired_position_.x=float(raw_input())
#	    	print 'enter y coordinate:'
#	    	desired_position_.y=float(raw_input())
#	    	print 'X: %s' % desired_position_.x
#	    	print 'Y: %s' % desired_position_.y
	    change_state(0)
	    fix_yaw(desired_position_)
            pass
        else:
            print 'state unknown'
            rospy.logerr('Unknown state!')
            pass
        rate.sleep()

if __name__ == '__main__':
    main()

#TODO add nodemap and pathfinding, port the nodemap into the destination functions.
#TODO add some object avoidance
#TODO add angle/travel hysterisis


