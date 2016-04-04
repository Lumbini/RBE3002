import math
import array


def heuristic(node1, node2):
	## calculate the shortest distance between two nodes
	return (math.sqrt(math.pow((node2[0]-node1[0]), 2) + math.pow((node2[1]-node1[1]), 2)))

## manhatten distance is just the sum of the difference in x and y
def manhattan(node1, node2):
	return (node1.x - node2.x) + (node1.y - node2.y)

## constructs the path
def constructPath(fromNode, pathIn):
	path = pathIn
    path.append(fromNode) ## put this node in the list

	if fromNode.parentNode is not None:
	    return constructPath(fromNode.parentNode, path) ## go to the parent and try again
	else:
	    pathIn.reverse()
	    return pathIn

## finds path
def AStar(start, goal):
	explored = []		## Nodes that need to be explored	
	frontiers = []		## Nodes that can be pathed through
	gScore = []			## Distance from the start of the node
	fScore = []			## Estimated distance from start to the end goal
	cameFrom = []		## Stores the parents of the nodes
	counter = 1			## Keep track of the elements in GScore and FScore

	## initializing the gScore and fScore indices to infinity
	gScore[:] = math.infinity
	fScore[:] = math.infinity

	gScore[start] = 0			
	fScore[start] = heuristic(start, goal)

	while explored is not empty:

		frontiers.sort() 	
		currentNode = frontiers[0]

		if currentNode == goal:
			return constructPath(currentNode, )		#add in params

		## Transfer current node from frontiers to explored 
		frontiers.pop([0])		
		explored.append(currentNode)


		for node in adjacentNodes:
			if node not in explored:
				## Calculatethe heuristic from current to new node, and add to the temp gScore list
				temp_gScore = gScore[start] + heuristic(currentNode, node)	

				if node not in frontiers:
					frontiers.append(node)

				elif temp_gScore < gScore[counter]:
					gScore[counter] = temp_gScore
					fScore[counter] = temp_gScore + heuristic(node, goal)
					cameFrom[counter] = currentNode
					counter += 1


					## UPDATE PATH

	return None

	## returns the path from start to end
	def constructPath(fromNode, pathIn): #start this by giving it a LocationNode and an empty list for path
        path = pathIn
        path.append(fromNode) #put this node in the list

        if fromNode.parentNode is not None:
            return constructPath(fromNode.parentNode, path) #go to the parent and try again
        else:
            pathIn.reverse()
            return pathIn










