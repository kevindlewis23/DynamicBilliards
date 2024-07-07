#define _USE_MATH_DEFINES


#include <cmath>


// Simulation accuracy constants
const double dx = 0.000001; // Used as distance when calculating derivatives
const int NUM_ITER = 1000; // Number of iterations when running binary search / Newton's method to approximate tangents


// STARTING CONDITIONS ARE IN THE MAIN FUNCTION


// Stub all the future functions and classes
class Point;
class Shape;
class Circle;
class Rectangle;
class Ellipse;
class Segment;
class ArbitraryShape;
double normalizeAngle(double angle);
double getAngleToPoint(Point point, const Shape& shape, double t);
bool angleToPointIncreasing(Point point, const Shape& shape, double t);
double* getTangentPoints(Point point, const Shape& shape);
double getBisectorAngle(Point point, const Shape& shape);
double getCollisionPoint(const Point& point, double angle, const Shape& shape);
void collideWithShape(const Point& point, double angle, const Shape& shape, Point& outMidpoint, Point& outFinalPoint, double& outNewAngle);
void oneIteration(const Point& point, double angle, const Shape& shape, Point& outMidpoint, Point& outFinalPoint, double& outNewAngle);
void collideWithShape(const Point& point, double angle, const Shape& shape, Point& outMidpoint, Point& outFinalPoint, double& outNewAngle);
void runIterations(const Point& startPoint, double startAngle, int n, const Shape& shape, std::vector<Point>& points);



// States for the drawing engine
enum class DrawState {
    WAITING_FOR_POINT,
    WAITING_FOR_ANGLE,
    DRAWING,
    PAUSED
};


bool killThread = false;

class Point {
public:
    double x;
    double y;

    Point() : x(0), y(0) {}

    Point(double x, double y) : x(x), y(y) {}

    static Point unit(double angle) {
        return Point(cos(angle), sin(angle));
    }

    double dist(const Point& other) const {
        return hypot(x - other.x, y - other.y);
    }

    double squareDist(const Point& other) const {
        return pow(x - other.x, 2) + pow(y - other.y, 2);
    }

    double L1Dist(const Point& other) const {
        return std::fabs(x - other.x) + std::fabs(y - other.y);
    }

    // Overload multiplication with a scalar (double)
    Point operator*(double scalar) const {
        return Point(x * scalar, y * scalar);
    }

    Point operator+(Point other) const {
        return Point(x + other.x, y + other.y);
    }

    Point operator-(Point other) const {
        return Point(x - other.x, y - other.y);
    }

    Point operator/(double scalar) const {
        return Point(x / scalar, y / scalar);
    }

};

// Shapes are defined by parametric equations that return points for t values
// All shapes must be loops, meaning the shape at maxT is the same as the shape at minT
// If your shape is not a loop, compose it with (1 - cosx)/2 or something like it to make it a loop
// THIS IS IMPORTANT, NON-LOOPING SHAPES MAY LEAD TO INFINITE LOOPS!!!!!!!
class Shape {
public:
    double maxT = 2 * M_PI;
    double minT = 0;
    // Parametric equation for the shape that takes values from minT to maxT and returns the point
    // Equations should be differentiable
    virtual Point equation(const double) const { return Point(0, 0); }

    // Derivative stuff
    // Get the tangent vector at a point, can replace with a close-form derivative
    virtual Point tangent(const double t) const {
        if (t > minT + dx && t < maxT - dx)
            return (equation(t + dx) - equation(t - dx)) / (2 * dx);
        if (t < minT + dx)
            return (equation(t + dx) - equation(t)) / dx;
        return (equation(t) - equation(t - dx)) / dx;
    }

    // Get the normal vector at a point 
    Point normal(const double t) const {
        Point tan = tangent(t);
        return Point(tan.y, -tan.x);
    }

    double normalizeT(double t) const {
        double ret = fmod(t - minT, maxT - minT) + minT;
        if (ret < minT)
            ret += maxT - minT;
        return ret;
    }

    // Add t and dt, accounting for looping shapes
    double addT(double t, double deltaT) const {
        return normalizeT(t + deltaT);
    }

    // Find the middle of two t values, accounting for looping shapes
    // Any two t values will have two middles on either side of the shape
    // This will get the one where increasing t takes you from t1 to the middle to t2
    double middleT(double t1, double t2) const {
        while (t2 < t1) t2 += maxT - minT;
        while (t2 > t1 + maxT - minT) t2 -= maxT - minT;
        return normalizeT((t1 + t2) / 2);
    }
};

class Circle : public Shape {
    double radius;
public:
    Circle(double radius) : radius(radius) {}

    Point equation(const double t) const override {
        return Point(radius * cos(t), radius * sin(t));
    }
};


// Stolen from https://math.stackexchange.com/questions/1958939/parametric-equation-for-rectangular-tubing-with-corner-radius
class Rectangle : public Shape {
    double a; // half width
    double b; // half height
    double r; // corner radius
public:
    Rectangle(const double halfWidth, const double halfHeight, double cornerRadius) {
        maxT = 8;
        a = halfWidth;
        b = halfHeight;
        r = cornerRadius;
    }
    // Recangle with rounded corners
    Point equation(const double tIn) const override {
        double t = fmod(tIn, 8);
        if (t < 1)
            return Point(a, (r - b) * (2 * t - 1));
        if (t < 2)
            return Point(a - r + r * cos(M_PI / 2 * (t - 1)), -b + r - r * sin(M_PI / 2 * (t - 1)));
        if (t < 3)
            return Point(-(a - r) * (2 * t - 5), -b);
        if (t < 4)
            return Point(-a + r - r * sin(M_PI / 2 * (t - 3)), -b + r - r * cos(M_PI / 2 * (t - 3)));
        if (t < 5)
            return Point(-a, (b - r) * (2 * t - 9));
        if (t < 6)
            return Point(-a + r - r * cos(M_PI / 2 * (t - 5)), b - r + r * sin(M_PI / 2 * (t - 5)));
        if (t < 7)
            return Point((a - r) * (2 * t - 13), b);
        return Point(a - r + r * sin(M_PI / 2 * (t - 7)), b - r + r * cos(M_PI / 2 * (t - 7)));

    }
};

class Ellipse : public Shape {
    double a; // semi-major axis
    double b; // semi-minor axis
public:
    Ellipse(double a, double b) : a(a), b(b) {}

    Point equation(const double t) const override {
        return Point(a * cos(t), b * sin(t));
    }
};

class Segment : public Ellipse {
public:
    Segment(const double width) : Ellipse(width, 0) {}
};


// Class for any general shape defined by a parametric equation
typedef Point(*input_function)(double);
class ArbitraryShape : public Shape {
    input_function eq;
    bool isLoop;
    double nonLoopingMinT;
    double nonLoopingPeriod;
public:
    // Takes a function that returns a point for a given t
    ArbitraryShape(input_function f, bool loop, double minArg, double maxArg) {
        eq = f;
        nonLoopingMinT = minArg;
        nonLoopingPeriod = maxArg - minArg;
        isLoop = loop;
        if (isLoop) {
            minT = minArg;
            maxT = maxArg;
        }
        else {
            minT = 0;
            maxT = 2 * M_PI;
        }
    }

    Point equation(const double t) const override {
        if (!isLoop) {
            // Loop from 0 to 1 back to 0
            double actualT = (1 - cos(t)) / 2;
            // Fit to given min and max t values
            actualT *= nonLoopingPeriod;
            actualT += nonLoopingMinT;
            return eq(actualT);
        }
        return eq(normalizeT(t));
    }
};

double normalizeAngle(double angle) {
    double ret = fmod(angle + M_PI, 2 * M_PI) - M_PI;
    if (ret < -M_PI)
        ret += 2 * M_PI;
    return ret;
}

double getAngleToPoint(Point point, const Shape& shape, double t) {
    Point point2 = shape.equation(t);
    // Angle of the vector from point to point2
    return atan2(point2.y - point.y, point2.x - point.x);
}

bool angleToPointIncreasing(Point point, const Shape& shape, double t) {
    double angle1 = getAngleToPoint(point, shape, shape.addT(t, -dx));
    double angle2 = getAngleToPoint(point, shape, shape.addT(t, dx));

    // Make sure these angles are close to each other
    if (angle2 - angle1 > M_PI)
        angle1 += 2 * M_PI;
    if (angle1 - angle2 > M_PI)
        angle2 += 2 * M_PI;
    return angle2 > angle1;
}


// Get the t value of the tangent points on the shape
// If the point is not on the outside of the shape, this could produce an infinite loop.
double* getTangentPoints(Point point, const Shape& shape) {
    // Get the min and max angles from the point to the shape

    // Assuming convexity, we have one region where angleToPoint is strictly increasing with respect to t,
    // and one region where it is strictly decreasing. We can binary search to find the boundary between these regions.

    // First, find a t value where the derivative is positive, and one where it's negative
    double incT = shape.minT;
    while (!angleToPointIncreasing(point, shape, incT)) {
        // Add a random amount to t between 0 and maxT-minT
        incT = shape.addT(incT, (double)rand() * (shape.maxT - shape.minT) / (double)RAND_MAX);
    }
    double decT = shape.minT;
    while (angleToPointIncreasing(point, shape, decT)) {
        decT = shape.addT(decT, (double)rand() * (shape.maxT - shape.minT) / (double)RAND_MAX);
    }

    // Use binary search to find the two boundaries
    double increasingT = incT;
    double decreasingT = decT;
    for (int i = 0; i < NUM_ITER; ++i) {
        // Note argument order matters for middleT
        double mid = shape.middleT(increasingT, decreasingT);
        if (angleToPointIncreasing(point, shape, mid))
            increasingT = mid;
        else
            decreasingT = mid;
    }

    double boundary1 = (increasingT + decreasingT) / 2;

    increasingT = incT;
    decreasingT = decT;
    for (int i = 0; i < NUM_ITER; ++i) {
        double mid = shape.middleT(decreasingT, increasingT);
        if (angleToPointIncreasing(point, shape, mid))
            increasingT = mid;
        else
            decreasingT = mid;
    }

    double boundary2 = (increasingT + decreasingT) / 2;

    // Return these two boundaries
    double* ret = new double[2];
    ret[0] = boundary1;
    ret[1] = boundary2;
    return ret;
}

double getBisectorAngle(Point point, const Shape& shape) {
    double* tVals = getTangentPoints(point, shape);
    double angle1 = getAngleToPoint(point, shape, tVals[0]);
    double angle2 = getAngleToPoint(point, shape, tVals[1]);
    delete[] tVals;
    return normalizeAngle((angle1 + angle2) / 2);

}


bool angleTooHigh(const Point& rayOrigin, double angle, const Shape& shape, double t) {
    double angToPoint = getAngleToPoint(rayOrigin, shape, t);
    return normalizeAngle(angToPoint - angle) > 0;
}

// Takes a point, shape, and angle
// Returns the t value of the collision point on the shape.
// Undefined behavior if the ray does not collide with the shape, but it will give some t value anyway.
double getCollisionPoint(const Point& point, double angle, const Shape& shape) {
    double* tangentVals = getTangentPoints(point, shape);
    // There will be two points that this ray collides with
    // We need to find both and return the one that is closest to the point
    double tooLow, tooHigh;
    if (angleTooHigh(point, angle, shape, tangentVals[0])) {
        tooLow = tangentVals[1];
        tooHigh = tangentVals[0];
    }
    else {
        tooLow = tangentVals[0];
        tooHigh = tangentVals[1];
    }
    delete[] tangentVals;
    double tl = tooLow;
    double th = tooHigh;

    for (int i = 0; i < NUM_ITER; ++i) {
        double mid = shape.middleT(tl, th);
        if (angleTooHigh(point, angle, shape, mid))
            th = mid;
        else
            tl = mid;
    }

    double mid1 = shape.middleT(tl, th);

    tl = tooLow;
    th = tooHigh;

    for (int i = 0; i < NUM_ITER; ++i) {
        double mid = shape.middleT(th, tl);
        if (angleTooHigh(point, angle, shape, mid))
            th = mid;
        else
            tl = mid;
    }
    double mid2 = shape.middleT(th, tl);

    // Check if the dist to mid1 or mid2 is greater
    double dist1 = shape.equation(mid1).L1Dist(point);
    double dist2 = shape.equation(mid2).L1Dist(point);
    if (dist1 < dist2)
        return mid1;
    return mid2;
}

// Transformation function, applying one iteration of the collision
void collideWithShape(const Point& point, double angle, const Shape& shape, Point& outMidpoint, Point& outFinalPoint, double& outNewAngle) {
    double t = getCollisionPoint(point, angle, shape);
    if (isnan(t)) {
        outMidpoint = point;
        outFinalPoint = point;
        outNewAngle = angle;
        return;
    }
    Point collisionPoint = shape.equation(t);
    Point tanVec = shape.tangent(t);
    // Reflect the angle about the tangent
    double newAngle = 2 * atan2(tanVec.y, tanVec.x) - angle;
    outMidpoint = collisionPoint;
    outFinalPoint = collisionPoint + Point(cos(newAngle), sin(newAngle)) * collisionPoint.dist(point);
    outNewAngle = newAngle + M_PI;
}

void oneIteration(const Point& point, double angle, const Shape& shape, Point& outMidpoint, Point& outFinalPoint, double& outNewAngle) {
    collideWithShape(point, angle, shape, outMidpoint, outFinalPoint, outNewAngle);
    outNewAngle = normalizeAngle(2 * getBisectorAngle(outFinalPoint, shape) - outNewAngle);
    // outNewAngle = getBisectorAngle(outFinalPoint, shape); <-- Uncomment this to find angle bisectors.
}

void killCalculation() {
	killThread = true;
}

void unkillCalculation() {
	killThread = false;
}

void runIterations(const Point& startPoint, double startAngle, int n, const Shape& shape, std::vector<Point>& points) {
    points.clear();
    points.push_back(startPoint);
    Point curPoint = startPoint;
    double curAngle = startAngle;
    for (int i = 0; i < n; ++i) {
        if (killThread) return;
        Point nextMidpoint, nextFinalPoint;
        oneIteration(curPoint, curAngle, shape, nextMidpoint, nextFinalPoint, curAngle);
        points.push_back(nextMidpoint);
        points.push_back(nextFinalPoint);
        curPoint = nextFinalPoint;
    }
}
