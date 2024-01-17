import math
import pygame
from pygame import gfxdraw
class Point:
    x=0
    y=0
    def __init__(self,x,y):
        self.x=x
        self.y=y
    # Initializer from angle
    def unit(angle):
        return Point(math.cos(angle),math.sin(angle))
    
    def __str__(self):
        return "({},{})".format(self.x,self.y)
    def __repr__(self):
        return "({},{})".format(self.x,self.y)
    
    # Math stuff
    def __add__(self,other):
        return Point(self.x+other.x,self.y+other.y)
    def __sub__(self,other):
        return Point(self.x-other.x,self.y-other.y)
    # Multiply a number by a point
    def __mul__(self,other):
        return Point(self.x*other,self.y*other)
    def __rmul__(self,other):
        return Point(self.x*other,self.y*other)
    # Euclidean distance
    def __abs__(self):
        return math.sqrt(self.x**2+self.y**2)
    # Distance between two points
    def dist(self,other):
        return abs(self-other)
    

class Line:
    a=Point(0,0)
    b=Point(0,0)
    def __init__(self,a,b):
        self.a=a
        self.b=b
    def __str__(self):
        return "{} -> {}".format(self.a,self.b)
    def __repr__(self):
        return "{} -> {}".format(self.a,self.b)

# Define constants
dx = .00001 # Used as distance when calculating derivatives
step = .01
NUM_ITER = 1000
MAX_ITER = 1000000

# Starting conditions
# distance will take a point and return a positive value if you are outside the boundary, negative if inside, and 0 if on
# This function will be used to take derivates and get tangents so try to make it smooth
# Circle centered at union
def circle (radius):
    return lambda point: point.x**2 + point.y**2 - 1

def ellipse (a, b, r):
    return lambda point: a.dist(point) + b.dist(point) - r

# sampleEllipse
distance = ellipse(Point(-1,0), Point(2,0), 3.5)
# distance = circle(1)


startPoint = Point(0,2)
# Angle from -1 to 1, -1 is furthest right of cone from the point's POV, 1 is furthest left
startAngle = 3.2 * math.pi/2

# Function to get the tangent angle of the shape at a point (on-ish the shape)
def getTangent(point, distFunc):
    # Get the distance of the point
    dist = distFunc(point)
    # Run binary search on angles between 0 and pi to see when the distFunc is closest to dist
    minAngle = 0
    minDeriv = distFunc(point + Point.unit(minAngle) * dx) / dx - dist / dx
    
    maxAngle = math.pi
    maxDeriv = distFunc(point + Point.unit(maxAngle) * dx) / dx - dist / dx
    # Loop for Num_Iter
    for i in range(NUM_ITER):
        # Get the midpoint
        midAngle = (minAngle + maxAngle) / 2
        midDeriv = distFunc(point + Point.unit(midAngle) * dx) / dx - dist / dx
        # If the derivative is positive, the angle is too big, so set the max to the mid
        if bool(midDeriv > 0) == bool(maxDeriv > minDeriv):
            maxAngle = midAngle
            maxDeriv = midDeriv
        # If the derivative is negative, the angle is too small, so set the min to the mid
        else:
            minAngle = midAngle
            minDeriv = midDeriv
    # Return the angle
    return (minAngle + maxAngle) / 2

# Get the point of a collision given there is a collision between two points
def getCollisionPoint (p1, p2, distFunc):
    # Run binary search to find the closest distance to 0
    minDist = distFunc(p1)
    minPoint = p1
    maxDist = distFunc(p2)
    maxPoint = p2
    # Loop for Num_Iter
    for i in range(NUM_ITER):
        # Get the midpoint
        midPoint = (minPoint + maxPoint) * .5
        midDist = distFunc(midPoint)
        # If the derivative is positive, the angle is too big, so set the max to the mid
        if bool(midDist > 0) == bool(maxDist > minDist):
            maxPoint = midPoint
            maxDist = midDist
        # If the derivative is negative, the angle is too small, so set the min to the mid
        else:
            minPoint = midPoint
            minDist = midDist
    # Return the midpoint
    return (minPoint + maxPoint) * .5


def percentToAngle(point, percent, distFunc):
    pass


def collideWithShape (point, angle, distFunc):
    unit = Point.unit(angle)
    # Step by the step count until distance is negative
    midpoint = Point(0,0)
    for i in range(MAX_ITER):
        curPoint = point + i * unit * step
        if (distFunc(curPoint) < 0):
            midpoint = getCollisionPoint(curPoint - unit * step, curPoint, distFunc)
            break
    # Get the tangent angle
    tangentAngle = getTangent(midpoint, distFunc)
    # Get the new heading angle from reflection
    newAngle = 2 * tangentAngle - angle
    # Get the length from point to midpoint
    dist = math.sqrt((point.x - midpoint.x)**2 + (point.y - midpoint.y)**2)
    # Get the final point from midpoint
    finalPoint = midpoint + Point.unit(newAngle) * dist
    return (midpoint, finalPoint)
   
endPoint = collideWithShape(startPoint, startAngle, distance)

# Drawing stuff
pygame.init()

w, h = 800, 600
centerx, centery = w/2, h/2

scalingFactor = 100
maxBright = 255

RED = (255, 0, 0)
BLACK = (0, 0, 0)
BLUE = (0, 255, 0)
GREEN = (0,0,255)
WHITE = (255,255,255)

pointWidth = 5

# Not used currently
lineWidth = 2


# Max distance to the main shape to draw
maxDist = .03

def lightToRGB(l):
    b = int(l * maxBright)
    return (b, b, b)
     

screen = pygame.display.set_mode((w,h))
done = False


# Draw the sampleEllipse by coloring each pixel based on how close the sampleEllipse function there is to 0
for x in range(w):
    for y in range(h):
        dist = distance(Point((x - centerx) / scalingFactor, (centery - y) / scalingFactor))
        if math.fabs(dist) < maxDist:
            brightness = 1 - (math.fabs(dist) / maxDist)
            screen.set_at((x,y), lightToRGB(brightness))
        else:
            screen.set_at((x,y), (0,0,0))

def pixLoc(point):
    return (int(point.x * scalingFactor + centerx), h - int(point.y * scalingFactor + centery))

def drawPoint (color, point):
    gfxdraw.aacircle(screen, *pixLoc(point), pointWidth, color)
    gfxdraw.filled_circle(screen, *pixLoc(point), pointWidth, color)

def drawLine (color, line):
    p1, p2 = line.a, line.b
    # Use line thickness later
    pygame.draw.aaline(screen, color, pixLoc(p1), pixLoc(p2))


drawLine(WHITE, Line(startPoint, endPoint[0]))
drawLine(WHITE, Line(endPoint[0], endPoint[1]))

drawPoint(RED, startPoint)
drawPoint(GREEN, endPoint[0])
drawPoint(BLUE, endPoint[1])



pygame.display.flip()
while not done:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True
