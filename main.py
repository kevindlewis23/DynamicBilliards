import math
import pygame
from pygame import gfxdraw
import time
from threading import Thread
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
    # Square distance between two points
    def squareDist(self,other):
        return (self.x - other.x)**2 + (self.y - other.y)**2

# distance will take a point and return a positive value if you are outside the boundary, negative if inside, and 0 if on
# This function will be used to take derivates and get tangents so try to make it smooth
# Circle centered at union
def circle (radius):
    return lambda point: point.x**2 + point.y**2 - radius**2

# Ellipse with a b as foci, r as radius
def ellipse (a, b, r):
    return lambda point: a.dist(point) + b.dist(point) - r

# Helper function for rectangle as lambdas can only be one line
# 0 if on the rectangle, positive if outside, negative if inside
def rectDistance (point, ul, br):
    t, b, l, r = ul.y, br.y, ul.x, br.x
    # Check if point is up left of top left
    if point.x < l and point.y > t:
        # Return the L1 distance
        return l - point.x + point.y - t
    # Check if point is up right of top right
    if point.x > r and point.y > t:
        # Return the distance
        return point.x - r + point.y - t
    # Check if point is down left of bottom left
    if point.x < l and point.y < b:
        # Return the distance
        return l - point.x + b - point.y
    # Check if point is down right of bottom right
    if point.x > r and point.y < b:
        # Return the distance
        return point.x - r + b - point.y

    # Check if point is outside the rectangle
    if point.x < l or point.x > r:
        # Get the distance to the closest point on the rectangle
        return min(abs(point.x - l), abs(point.x - r))
    if point.y < b or point.y > t:
        # Get the distance to the closest point on the rectangle
        return min(abs(point.y - t), abs(point.y - b))
    
    # Get the distance to the closest point on the rectangle
    dist = min(abs(point.x - l), abs(point.x - r), abs(point.y - t), abs(point.y - b))
    # Return the distance
    return -dist

# Rectangle with ul upper-left point, br bottom-right point
def rectangle (ul, br):
    return lambda point: rectDistance(point, ul, br)

# ---------------------------------------- INITIAL CONDITIONS -------------------

# This is where the shape is defined.  It is a function that takes a point in the plane
# and returns 0 if the point is on the shape, a negative value if it is in the shape, 
# and a positive value if it is out of the shape.  I may try to generalize to parametric equations later.
# This function must be somewhat smooth (at least near the boundary) as it will be used to find tangents.
distance = ellipse(Point(-1,0), Point(2,0), 3.5)
# distance = circle(1)
# distance = rectangle(Point(-1, 1), Point(1, -1))

# Point to start at
startPoint = Point(0,2)

# Angle from -1 to 1, -1 is furthest right of cone from the point's POV, 1 is furthest left
startAngle = .9

# If you want the start angle to actually be an angle in radians, set this to false
startAngleIsPercent = True

# Any angle that points toward the shape, just used for initial calculations, DON'T FORGET TO DO THIS! 
# Not used if using an actual angle in radians rather than a percent
validAngle = 3*math.pi/2

# ------------------------------------ SIMULATION ACCURACY CONSTANTS ------------
dx = .00001 # Used as distance when calculating derivatives
step = .01 # Small step size when raycasting towards the shape
large_step = .5 # large step size when raycasting towards the shape
large_step_dist = 1 # Minimum value of the "distance" function to take large steps 
NUM_ITER = 50 # Number of iterations when running binary search to approximate tangents
MAX_ITER = 1000  # Maximum number of steps when raycasting.  If you reach this number, we
                # assume the ray is not intersecting the shape
# ------------------------------------ DRAWING CONDITIONS -------------------------

# screen size
w, h = 800, 600

# How big to scale from plane to screen
scalingFactor = 100

# Sizeof a point
pointWidth = 5

# Max distance to the main shape to draw (using distance function)
maxDist = .03

# Time between drawing points/lines
draw_delay = 0.1

# -------------------------------------------------------------------------------

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

def normalizeAngle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

# Note that percent is actually from -1 to 1
def percentToAngle(point, percent, distFunc, validAngle):
    angles = getMinMaxAngles(point, distFunc, validAngle)
    return angles[0] + (angles[1] - angles[0]) * (percent + 1) / 2
    
def angleToPercent(point, angle, distFunc):
    angles = getMinMaxAngles(point, distFunc, angle)
    return 2 * (angle - angles[0]) / (angles[1] - angles[0]) - 1

def rayIntersects(point, angle, distFunc):
    # Check if any location on the ray has a negative distFunc
    curPoint = point
    unit = Point.unit(angle)
    lastVal = math.inf
    # Step by the step count until distance is negative
    for i in range(MAX_ITER):
        dist = distFunc(curPoint)
        if (dist < 0):
            return True
        # Check if distance is increasing
        if (dist > lastVal):
            return False
        lastVal = dist
        if (dist > large_step_dist) :
            curPoint = curPoint + unit * large_step
        else:
            curPoint = curPoint + unit * step
    return False

def getMinMaxAngles(point, distFunc, validAngle):
    # Get the minimum and max angles to tangents with the shape using binary search
    # Valid angle can be used a starting point because it will definitely point towards the shape
    # Assuming the shape is convex
    tooLow = validAngle - math.pi/2
    tooHigh = validAngle
    # Get the lowest angle with binary search
    for i in range(NUM_ITER):
        midAngle = (tooLow + tooHigh) / 2
        # Check if the line from the point to the midangle intersects with the shape
        if rayIntersects(point, midAngle, distFunc):
            tooHigh = midAngle
        else:
            tooLow = midAngle
    minAngle = (tooLow + tooHigh) / 2

    tooLow = validAngle
    tooHigh = validAngle + math.pi/2
    # Get the highest angle with binary search
    for i in range(NUM_ITER):
        midAngle = (tooLow + tooHigh) / 2
        # Check if the line from the point to the midangle intersects with the shape
        if rayIntersects(point, midAngle, distFunc):
            tooLow = midAngle
        else:
            tooHigh = midAngle
    maxAngle = (tooLow + tooHigh) / 2
    return (minAngle, maxAngle)

def collideWithShape (point, angle, distFunc):
    unit = Point.unit(angle)
    # Step by the step count until distance is negative
    midpoint = Point(0,0)
    curPoint = point
    for i in range(MAX_ITER):
        dist = distFunc(curPoint)
        if (dist < 0):
            midpoint = getCollisionPoint(curPoint - unit * step, curPoint, distFunc)
            break
        if dist > large_step_dist:
            curPoint = curPoint + unit * large_step
        else:
            curPoint = curPoint + unit * step
    # Get the tangent angle
    tangentAngle = getTangent(midpoint, distFunc)
    # Get the new heading angle from reflection
    newAngle = 2 * tangentAngle - angle
    # Get the length from point to midpoint
    dist = math.sqrt((point.x - midpoint.x)**2 + (point.y - midpoint.y)**2)
    # Get the final point from midpoint
    finalPoint = midpoint + Point.unit(newAngle) * dist
    return (midpoint, finalPoint, normalizeAngle(newAngle + math.pi))

def runIterations(point, startAngle, n, distFunc, points):
    # Clear points list
    points.clear()
    points.append(point)
    curPoint = point
    curAngle = startAngle
    for i in range(n):
        nextPoints = collideWithShape(curPoint, curAngle, distFunc)
        points.append(nextPoints[0])
        points.append(nextPoints[1])
        if i < n - 1:
            curPoint = nextPoints[1]
            curPercent = angleToPercent(curPoint, nextPoints[2], distFunc)
            curAngle = percentToAngle(curPoint, -curPercent, distFunc, nextPoints[2])

def runFromPercent(point, percent, validAngle, n, distFunc, points):
    # Get the angle
    angle = percentToAngle(point, percent, distFunc, validAngle)
    # Run the iterations
    runIterations(point, angle, n, distFunc, points)

# Drawing stuff
pygame.init()


centerx, centery = w/2, h/2
maxBright = 255


def lightToRGB(l):
    b = int(l * maxBright)
    return (b, b, b)

def hslToRGB(h, s, l):
    c = (1 - abs(2*l - 1)) * s
    x = c * (1 - abs((h/60) % 2 - 1))
    m = l - c/2
    if h < 60:
        r, g, b = c, x, 0
    elif h < 120:
        r, g, b = x, c, 0
    elif h < 180:
        r, g, b = 0, c, x
    elif h < 240:
        r, g, b = 0, x, c
    elif h < 300:
        r, g, b = x, 0, c
    else:
        r, g, b = c, 0, x
    return (int((r+m)*maxBright), int((g+m)*maxBright), int((b+m)*maxBright))

screen = pygame.display.set_mode((w,h))
done = False




def pixLoc(point):
    return (int(point.x * scalingFactor + centerx), h - int(point.y * scalingFactor + centery))

def drawPoint (color, point):
    gfxdraw.aacircle(screen, *pixLoc(point), pointWidth, color)
    gfxdraw.filled_circle(screen, *pixLoc(point), pointWidth, color)

def drawLine (color, p1, p2):
    # Use line thickness later
    pygame.draw.aaline(screen, color, pixLoc(p1), pixLoc(p2))

curN = 8
last_draw = -1
draw_index = 0

isDrawing = False

pointsArr = []
nextPointsArr = []

def createThread():
    if startAngleIsPercent:
        t = Thread(target=runFromPercent, args=(startPoint, startAngle, validAngle, curN, distance, nextPointsArr))
        t.start()
        return t
    else:
        t = Thread(target=runIterations, args=(startPoint, startAngle, curN, distance, nextPointsArr))
        t.start()
        return t

calcthread = createThread()
while not done:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True

    if not isDrawing:
        # Check if the calculation thread is done
        if not calcthread.is_alive():
            # Get the result from the thread
            calcthread.join()
            pointsArr = nextPointsArr
            nextPointsArr=[]
            isDrawing = True
            last_draw = -1
            draw_index = 0

                # Clear the screen
            screen.fill((0,0,0))
            # Draw the sampleEllipse by coloring each pixel based on how close the sampleEllipse function there is to 0
            for x in range(w):
                for y in range(h):
                    dist = distance(Point((x - centerx) / scalingFactor, (centery - y) / scalingFactor))
                    if math.fabs(dist) < maxDist:
                        brightness = 1 - (math.fabs(dist) / maxDist)
                        screen.set_at((x,y), lightToRGB(brightness))
                    else:
                        screen.set_at((x,y), (0,0,0))
            pygame.display.flip()

            # Restart the calculation with double the n
            curN *= 2
            calcthread = createThread()
    else:
        # Check if we're done drawing
        if draw_index >= len(pointsArr):
            isDrawing = False
            continue

        # Check if it's time to draw
        if time.time() - last_draw >= draw_delay:
            color = hslToRGB(300 * draw_index / len(pointsArr),1,0.5)
            drawPoint(color, pointsArr[draw_index])
            if draw_index > 0:
                drawLine(color, pointsArr[draw_index-1], pointsArr[draw_index])
            pygame.display.flip()
            draw_index += 1
            last_draw = time.time()


        
    
    