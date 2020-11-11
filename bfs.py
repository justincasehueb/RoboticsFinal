# visits all the nodes of a graph (connected component) using BFS
graph = {'0': ['2', '3', '1'],
         '1': ['0','7'],
         '2': ['0'],
         '3': ['4','0'],
         '4': ['5', '6','3'],
         '5': ['4'],
         '6': ['4','7'],
         '7': ['6','1']	}

# finds shortest path between 2 nodes of a graph using BFS
def bfs_shortest_path(graph, start, goal):
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
 
print("Please input the starting node: ")
#for python 3.x raw_input() is deprecated, use input() lines
x=raw_input() 
#x=input()
print("Please input the destination: ")
y=raw_input()
#y=input()
print(bfs_shortest_path(graph, x, y))

#adapted from SOURCE https://pythoninwonderland.wordpress.com/2017/03/18/how-to-implement-breadth-first-search-in-python/
