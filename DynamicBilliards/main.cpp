#define _USE_MATH_DEFINES


#include <iostream>
#include <cmath>
#include <vector>
#include <SFML/Graphics.hpp>
#include <thread>
#include <limits>


// Simulation accuracy constants
const double dx = 0.000001; // Used as distance when calculating derivatives
const double step = 0.0003; // Small step size when raycasting towards the shape
const double large_step = 0.3; // large step size when raycasting towards the shape
const double grad_descent_rate = .1; // Rate of gradient descent when approximating collisions
const double large_step_dist = .4; // Minimum distance from the shape to take large steps 
const int NUM_ITER = 1000; // Number of iterations when running binary search / Newton's method to approximate tangents
const int POLLING_POINTS = 10; // Number of points to start from when doing Newton's method
const double EPSILON = .001; // Something near 0

// Drawing constants
const int pointRadius = 5;
double scalingFactor = 200; // How big to scale from plane to screen
double maxDist = 0.03; // Max distance to the main shape to draw
int numPointsOfShape = 1000; // Number of points on the shape to draw

// Window size
int w = 2000;
int h = 1500;

// Minimum Time between drawing points / lines in seconds
// Set to 0 for delay solely due to calculation
double draw_delay = 0.0;

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
void drawPoint(sf::RenderWindow& window, const Point& point, double scalingFactor, sf::Color color);
void drawLine(sf::RenderWindow& window, const Point& p1, const Point& p2, double scalingFactor, sf::Color color);
void collideWithShape(const Point& point, double angle, const Shape& shape, Point& outMidpoint, Point& outFinalPoint, double& outNewAngle);
void runIterations(const Point& startPoint, double startAngle, int n, const Shape& shape, std::vector<Point>& points);


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
		return fmod(t - minT, maxT - minT) + minT;
	}

    // Add t and dt, accounting for looping shapes
    double addT(double t, double deltaT) const {
		return fmod(t + deltaT - minT, maxT - minT) + minT;
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

class Rectangle : public Shape {
    Point ul; // upper-left point
    Point br; // bottom-right point
public:
    Rectangle(const Point& ul, const Point& br) : ul(ul), br(br) {
        maxT = 4;
    }
    Point equation(const double t) const override {
        if (t < 1)
			return Point(ul.x, ul.y + (br.y - ul.y) * t);
        if (t < 2)
            return Point(ul.x + (br.x - ul.x) * (t - 1), br.y);
        if (t < 3)
            return Point(br.x, br.y - (br.y - ul.y) * (t - 2));
        return Point(br.x - (br.x - ul.x) * (t - 3), ul.y);

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
typedef Point (*input_function)(double);
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
            double actualT = (1-cos(t)) / 2;
            // Fit to given min and max t values
            actualT *= nonLoopingPeriod;
            actualT += nonLoopingMinT;
            return eq(actualT);
        }
		return eq(t);
	}
};

double normalizeAngle(double angle) {
    return fmod(angle + M_PI, 2 * M_PI) - M_PI;
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

void runIterations(const Point& startPoint, double startAngle, int n, const Shape& shape, std::vector<Point>& points) {
    points.clear();
    points.push_back(startPoint);
    Point curPoint = startPoint;
    double curAngle = startAngle;
    for (int i = 0; i < n; ++i) {
        if (killThread) return;
        Point nextMidpoint, nextFinalPoint;
        double nextAngle;
        collideWithShape(curPoint, curAngle, shape, nextMidpoint, nextFinalPoint, nextAngle);
        points.push_back(nextMidpoint);
        points.push_back(nextFinalPoint);

        if (i < n - 1) {
            curPoint = nextFinalPoint;
            // Reflect the out angle about the bisector angle
            curAngle = 2 * getBisectorAngle(nextFinalPoint, shape) - nextAngle;
        }
    }
}


void drawPoint(sf::RenderWindow& window, const Point& point, double scalingFactor, sf::Color color) {
    sf::CircleShape sfmlShape(pointRadius);
    sfmlShape.setFillColor(color);
    sfmlShape.setPosition(point.x * scalingFactor + window.getSize().x * 0.5, -point.y * scalingFactor + 0.5 * window.getSize().y);
    // Center the circle
    sfmlShape.move(-pointRadius, -pointRadius);
    window.draw(sfmlShape);
}

void drawLine(sf::RenderWindow& window, const Point& p1, const Point& p2, double scalingFactor, sf::Color color) {
    sf::Vertex line[] = {
        sf::Vertex(sf::Vector2f(p1.x * scalingFactor + window.getSize().x * 0.5, -p1.y * scalingFactor + 0.5 * window.getSize().y)),
        sf::Vertex(sf::Vector2f(p2.x * scalingFactor + window.getSize().x * 0.5, -p2.y * scalingFactor + 0.5 * window.getSize().y))
    };
    // Draw the line the correct color
    line[0].color = color;
    line[1].color = color;
    window.draw(line, 2, sf::Lines);
}

// HSL to RGB conversion
// https://en.wikipedia.org/wiki/HSL_and_HSV#From_HSL
void HSLtoRGB(double h, double s, double l, double& r, double& g, double& b) {
    double c = (1 - std::fabs(2 * l - 1)) * s;
    double x = c * (1 - std::fabs(fmod(h / 60, 2) - 1));
    double m = l - c / 2;
    if (h < 60) {
        r = c;
        g = x;
        b = 0;
    }
    else if (h < 120) {
        r = x;
        g = c;
        b = 0;
    }
    else if (h < 180) {
        r = 0;
        g = c;
        b = x;
    }
    else if (h < 240) {
        r = 0;
        g = x;
        b = c;
    }
    else if (h < 300) {
        r = x;
        g = 0;
        b = c;
    }
    else {
        r = c;
        g = 0;
        b = x;
    }
    r += m;
    g += m;
    b += m;
}

int main() {

    // Example usage
    //  Circle shape(1);
    // Create a rectangle
    // Rectangle shape(Point(-1, 0), Point(1, -0.02));
    // Create an ellipse
    // Ellipse shape(Point(-1, 0), Point(1, 0), 2.5);
    /*
    double r = sqrt(3);
    double theta = 1.4 * M_PI / 4 + 0.1;
    Point startPoint(r * cos(theta), r * sin(theta));
    double startAngle = atan2(-startPoint.y, -startPoint.x);
    */

 

    Ellipse shape(1.3, .6);
    // Circle shape(1.2);

    Point startPoint = Point(-1, 2);

    double startAngle = -1.72;

    //Segment shape(1);
    //Point startPoint = Point(-sqrt(2)/2, sqrt(2)/2);
    //// Angle from start point to the origin
    //double startAngle = atan2(-startPoint.y, -startPoint.x);
    

    // Create the calculation thread
    std::vector<Point> points;
    std::thread calculationThread;
    calculationThread = std::thread([&] {
        runIterations(startPoint, startAngle, 100000, shape, points);
    });




    // Create the window

    sf::RenderWindow window(sf::VideoMode(w, h), "SFML Drawing");
    int numDrawn = 1;

    // Pixel array for the shape
    sf::VertexArray curve;
    bool recalculatePixels = true;
    // Main loop
    while (window.isOpen()) {
        // Event handling
        sf::Event event;
        if (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                // Kill the calculation thread
                killThread = true;
                calculationThread.join();
                window.close();
            }
            // On resize, edit w and h
            if (event.type == sf::Event::Resized) {
                w = event.size.width;
                h = event.size.height;
                recalculatePixels = true;
            }
        }

        window.clear(sf::Color::Black);


        // Draw the sample shape
        if (recalculatePixels) {
            recalculatePixels = false;
            // Clear the curve
            curve.clear();
            curve.setPrimitiveType(sf::LineStrip);
            curve.resize(numPointsOfShape + 1);
            for (int i = 0; i < numPointsOfShape; ++i) {
				double t = shape.minT + (shape.maxT - shape.minT) * i / numPointsOfShape;
				Point point = shape.equation(t);
                curve[i].position = sf::Vector2f(point.x * scalingFactor + window.getSize().x * 0.5, -point.y * scalingFactor + 0.5 * window.getSize().y);
			}
            curve[numPointsOfShape].position = curve[0].position;
        }
        window.draw(curve);


        // Check if points has more points than numDrawn
        if (points.size() > numDrawn) {
            numDrawn ++;
            // Draw the points and lines
            for (int i = 0; i < numDrawn; ++i) {
                double r = 0, g = 0, b = 0;
                HSLtoRGB(300 * i / numDrawn, 1, 0.5, r, g, b);
                sf::Color color(r * 255, g * 255, b * 255);
                drawPoint(window, points[i], scalingFactor, color);
                if (i > 0)
                    drawLine(window, points[i - 1], points[i], scalingFactor, color);
            }

            // Window only displays if more points are drawn
            window.display();
        }

        // Sleep for draw_delay
        std::this_thread::sleep_for(std::chrono::milliseconds((int)(draw_delay * 1000)));
    }

    return 0;
}
