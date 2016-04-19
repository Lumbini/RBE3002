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
		return (pow(self.x, 17) + pow(self.y, 19))