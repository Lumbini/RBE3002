import math

class Node:

    ## constructor
    def __init__(self, X, Y, probability, mapWidth):
        self.x = X
        self.y = Y
        self.data = probability
        self.parent = None
        self.cost = 10000000
        self.width = mapWidth

    ## returns a node from the grid
    def getNode(self, x, y, grid):
        return grid[x + y * self.width]

    ## returns an array of neighbors to the node on the grid
    def getNeighbors(self, grid):

        neighbors = []
        #print "in neighbors"
        ## for each x and y in the grid
        for x in range(int(self.x - 1), int(self.x + 2)):
            for y in range(int(self.y - 1), int(self.y + 2)):
                ## if that coordinate is within the grid and is not self
                if (x >= 0 and x < self.width) and (y >= 0 and y < (len(grid) / self.width)) and (x != self.x or y != self.y):
                    ## get that node
                    tempNode = self.getNode(x, y, grid)
                    #print "node in grid ", x, y, tempNode.data
                    ## if the node is not null, is not a wall, and is not unknown
                    if tempNode != None and tempNode.data != 100 and tempNode.data != -1:
                        ## append this node to our neighbors list
                        #print "node is neighbor"
                        neighbors.append(tempNode)

        ## return the neighbors list
        return neighbors

    def getAllNeighbors(self, grid):

        neighbors = []
        #print "in neighbors"
        ## for each x and y in the grid
        for x in range(int(self.x - 1), int(self.x + 2)):
            for y in range(int(self.y - 1), int(self.y + 2)):
                ## if that coordinate is within the grid and is not self
                if (x >= 0 and x < self.width) and (y >= 0 and y < (len(grid) / self.width)) and (x != self.x or y != self.y):
                    ## get that node
                    tempNode = self.getNode(x, y, grid)
                    #print "node in grid ", x, y, tempNode.data
                    ## if the node is not null, is not a wall, and is not unknown
                    if tempNode != None and tempNode.data != -1:

                        ## append this node to our neighbors list
                        #print "node is neighbor"
                        neighbors.append(tempNode)

        ## return the neighbors list
        return neighbors

    ## the equal function to check if two Nodes are equal
    ## basically need to check if class is the same and the coords are the same
    def __eq__(self, other):
        if isinstance(other, self.__class__):
            if self.x == other.x and self.y == other.y:
                return True
            else:
                return False
        else:
            return False

    ## the str function to informally print the node regularly
    def __str__(self):
        return "Node %d %d %d" % (self.x, self.y, self.data)

    ## the repr function to formally print the node regularly
    def __repr__(self):
        return "Node %d %d %d" % (self.x, self.y, self.data)

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
    neighbors = t3.getNeighbors(gridArr)
    for x in neighbors:
        print x
