import math
import array
from Node import Node
import sys

def heuristic(node1, node2):
	## calculate the shortest distance between two nodes
	return (math.sqrt(math.pow((node2[0]-node1[0]), 2) + math.pow((node2[1]-node1[1]), 2)))

## manhattan distance is just the sum of the difference in x and y
def manhattan(node1, node2):
	return abs((node1.x - node2.x)) + abs((node1.y - node2.y))

## constructs the path
def constructPath(fromNode):
	path = []
	currentNode = fromNode

	## traverse the parents to construct the list in backwards order
	while currentNode.parent is not None:
		path.append(currentNode)
		currentNode = currentNode.parent
	path.append(currentNode)
	print "path is ", path
	return path
	## reverse the list
	final = []
	for x in reversed(path):
		final.append(x)
	print "final path is ", final
	return final

## A* algorithm
def AStar(start, goal, grid):

	## explored nodes and nodes to be explored
	explored = []	
	frontiers = [start]

	## cost for start is 0
	start.cost = 0

	## while frontiers is not empty
	while len(frontiers) > 0:

		## sort frontiers and take the shortest'
		## TODO need to find a way to sort this based on manhattan distance
		#frontiers.sort()
		frontiers = sorted(frontiers, key=lambda Node: Node.cost)	
		currentNode = frontiers[0]

		## if we're at the goal, return the path
		if currentNode.x == goal.x and currentNode.y == goal.y:
			print "ABOUT TO CONSTRUCT THE PATH"
			return constructPath(currentNode)

		## Transfer current node from frontiers to explored 
		frontiers.pop(0)		
		explored.append(currentNode)

		neighbors = currentNode.getNeighbors(grid)
		## go through the neighbors of the currentNode, trying to find the best next option to add to frontiers
		for node in neighbors:
			print "neighbors array: ", neighbors
			if node.parent == None and node not in explored:
				## Calculate the cost from current to new node and update the new node's parent
				node.cost = currentNode.cost + manhattan(currentNode, node)	
				node.parent = currentNode
				frontiers.append(node)
			## else, new node has a parent... check if this cost is better than previous time we reached this node
			## if it is, update the cost and parent to match
			elif (currentNode.cost + manhattan(currentNode, node)) < node.cost:
				node.cost = manhattan(currentNode, node) + currentNode.cost
				node.parent = currentNode
				frontiers.append(node)

	## if we reach the end with no return from constructPath, a path does not exist
	return None


## main for testing purposes
if __name__ == "__main__":
         
    ## test grid layout         
    ## 0  1   2   3
    ## 4  5   6   7
    ## 8  9  10  11

    t1 = Node(0, 0, 0, 4) 
    t2 = Node(0, 1, 1, 4)  
    t3 = Node(0, 2, 2, 4)  
    t4 = Node(0, 3, 3, 4)
    t5 = Node(1, 0, 4, 4) 
    t6 = Node(1, 1, 5, 4) 
    t7 = Node(1, 2, 6, 4) 
    t8 = Node(1, 3, 7, 4)
    t9 = Node(2, 0, 8, 4)
    t10 = Node(2, 1, 9, 4)
    t11 = Node(2, 2, 10, 4)
    t12 = Node(2, 3, 11, 4)

    ## very important for the order here... otherwise finding neighbors will break
    gridArr = [t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12]
    path = AStar(t4, t12, gridArr)
    print path
