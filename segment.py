import math

l = -1
r = 1

t=math.pi/2+1
radius = math.sqrt(3)
startx = radius * math.cos(t)
h= radius * math.sin(t)
starta = math.atan(-startx/h)

def transformOnce(x, a): 
    newX = x + 2*h*math.tan(a)
    newA = a + math.atan((l-newX)/h) + math.atan((r-newX)/h)
    # print(math.atan((l-x)/h))
    # print(math.atan((r-x)/h))

    return (newX, newA)

def transform(x,a,n=1):
    if n == 0: return (x,a)
    return transform(*transformOnce(x,a),n-1)

print(transform(startx, starta, 0))
print(transform(startx, starta, 3))