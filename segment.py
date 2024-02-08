import math

l = -1
r = 1

t=math.pi/2-1
startx = math.cos(t)
h =  math.sin(t)
starta = 0

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
print(transform(startx, starta, 1))
print(transform(startx, starta, 2))
print(transform(startx, starta, 3))
print(transform(startx, starta, 4))