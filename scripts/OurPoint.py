import math

class OurPoint:

	## constructor for points
	def __init__(self, x, y):
		self.x = x
		self.y = y

	## overriding the equals function so we can compare two Points
	def __eq__(self, other):
	    if isinstance(other, self.__class__):
	        if self.x == other.x and self.y == other.y:
	            return True
	        else:
	            return False
	    else:
	        return False

	def __hash__(self):
		return hash(pow(self.x, 4) + pow(self.y, 6))

	def __str__(self):
	 	return "Point %d %d" % (self.x, self.y)

