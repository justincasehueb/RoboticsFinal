#import rospy
import math
from math import atan2

nodes = [[0,1,2,3],[1,0],[2,0],[3,0,4],[4,5,3,6],[5,4],[6,4]]
path = list()
checked = list()

def findPath(start,destination):
	print("pathing",start,destination)
	#we always start at start, obviously
	path.append(start)

	#if we are already at the destination, just return it.
	if start == destination:
		return path

	for child in nodes[start]:
		if child == destination:
			path.append(child)
			return path
	for child in nodes[start]:
		if child not in path:
			print(child,destination,path)
			findPath(child,destination)
		#else:
			#print("inpath:",child)


print(findPath(0,4))


	


