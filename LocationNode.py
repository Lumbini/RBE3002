class LocationNode:

	parentNode # another LocationNode
    adjacentNodes = []  # a set of adjacent LocationNodes
    data    # whatever data type is returned per-cell from the messages

    def __init__(self):
        self.data




class Path: 

    def ConstructPath(fromNode, pathIn): #start this by giving it a LocationNode and an empty list for path
        path = pathIn
        path.append(fromNode) #put this node in the list

        if fromNode.parentNode is not None:
            return ClimbHeirarchy(fromNode.parentNode, path) #go to the parent and try again
        else:
            pathIn.reverse()
            return pathIn
