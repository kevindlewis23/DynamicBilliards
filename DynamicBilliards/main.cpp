#define _USE_MATH_DEFINES


#include <iostream>
#include <cmath>
#include <vector>
#include <SFML/Graphics.hpp>
#include <thread>


// Simulation accuracy constants
const double dx = 0.00001; // Used as distance when calculating derivatives
const double step = 0.001; // Small step size when raycasting towards the shape
const double large_step = 0.3; // large step size when raycasting towards the shape
const double large_step_dist = .6; // Minimum value of the "distance" function to take large steps 
const int NUM_ITER = 100; // Number of iterations when running binary search to approximate tangents
const int MAX_ITER = 5000; // Maximum number of steps when raycasting.

// Drawing constants
const int pointRadius = 5;
double scalingFactor = 200; // How big to scale from plane to screen
double maxDist = 0.03; // Max distance to the main shape to draw

// Window size
int w = 2000;
int h = 1500;

// Minimum Time between drawing points / lines in seconds
double draw_delay = 0.01;

// STARTING CONDITIONS ARE IN THE MAIN FUNCTION


// Stub all the future functions and classes
class Point;
class Shape;
class Circle;
class Rectangle;
class Ellipse;
class NEllipse;
double normalizeAngle(double angle);
double percentToAngle(const Point& point, double percent, const Shape& shape, double validAngle);
double angleToPercent(const Point& point, double angle, const Shape& shape);
Point getCollisionPoint(const Point& p1, const Point& p2, const Shape& shape);
double* getMinMaxAngles(Point point, const Shape& shape, double validAngle);
bool rayIntersects(const Point& point, double angle, const Shape& shape);
void drawPoint(sf::RenderWindow& window, const Point& point, double scalingFactor, sf::Color color);
void drawLine(sf::RenderWindow& window, const Point& p1, const Point& p2, double scalingFactor, sf::Color color);
void collideWithShape(const Point& point, double angle, const Shape& shape, Point& outMidpoint, Point& outFinalPoint, double& outNewAngle);
void runIterations(const Point& startPoint, double startAngle, int n, const Shape& shape, std::vector<Point>& points);
void runFromPercent(const Point& startPoint, double percent, const Point& validPoint, int n, const Shape& shape, std::vector<Point>& points);


bool killThread = false;

class Point {
public:
    double x;
    double y;

    Point () : x(0), y(0) {}

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

};

class Shape {
public:
    virtual double distance(const Point& point) const = 0;
};

class Circle : public Shape {
    double radius;
public:
    Circle(double radius) : radius(radius) {}

    double distance(const Point& point) const override {
        return pow(point.x, 2) + pow(point.y, 2) - pow(radius, 2);
    }
};

class Rectangle : public Shape {
    Point ul; // upper-left point
    Point br; // bottom-right point
public:
    Rectangle(const Point& ul, const Point& br) : ul(ul), br(br) {}

    double distance(const Point& point) const override {
        double t = ul.y, b = br.y, l = ul.x, r = br.x;
        if (point.x < l && point.y > t)
            return l - point.x + point.y - t;
        if (point.x > r && point.y > t)
            return point.x - r + point.y - t;
        if (point.x < l && point.y < b)
            return l - point.x + b - point.y;
        if (point.x > r && point.y < b)
            return point.x - r + b - point.y;
        if (point.x < l || point.x > r)
            return std::min(fabs(point.x - l), fabs(point.x - r));
        if (point.y < b || point.y > t)
            return std::min(fabs(point.y - t), fabs(point.y - b));
        return -std::min(fabs(point.x - l), std::min(fabs(point.x - r), std::min(fabs(point.y - t), fabs(point.y - b))));
    }
};

class Ellipse : public Shape {
    Point a; // focus
    Point b; // focus
    double r; // radius
public:
    Ellipse(const Point& a, const Point& b, double r) : a(a), b(b), r(r) {}

    double distance(const Point& point) const override {
        return a.dist(point) + b.dist(point) - r;
    }
};

class NEllipse : public Shape {
    std::vector<Point> points;
    double r; // radius
public:
    NEllipse(const std::vector<Point>& points, double r) : points(points), r(r) {}

    double distance(const Point& point) const override {
        double sumDist = 0.0;
        for (const auto& p : points)
            sumDist += point.dist(p);
        return sumDist - r;
    }
};

double normalizeAngle(double angle) {
    return fmod(angle + M_PI, 2 * M_PI) - M_PI;
}

double* getMinMaxAngles(Point point, const Shape& shape, double validAngle) {
    double angles[2];
    // Get the minimum and maximum angles to tangents with the shape using binary search
    // Assuming the shape is convex
    double tooLow = validAngle - M_PI;
    double tooHigh = validAngle;
    // Get the lowest angle with binary search
    for (int i = 0; i < NUM_ITER; ++i) {
        double midAngle = (tooLow + tooHigh) / 2;
        // Check if the line from the point to the midangle intersects with the shape
        if (rayIntersects(point, midAngle, shape))
            tooHigh = midAngle;
        else
            tooLow = midAngle;
    }
    angles[0] = (tooLow + tooHigh) / 2;

    tooLow = validAngle;
    tooHigh = validAngle + M_PI;
    // Get the highest angle with binary search
    for (int i = 0; i < NUM_ITER; ++i) {
        double midAngle = (tooLow + tooHigh) / 2;
        // Check if the line from the point to the midangle intersects with the shape
        if (rayIntersects(point, midAngle, shape))
            tooLow = midAngle;
        else
            tooHigh = midAngle;
    }
    angles[1] = (tooLow + tooHigh) / 2;

    return angles;
}

double percentToAngle(const Point& point, double percent, const Shape& shape, double validAngle) {
    double* angles = getMinMaxAngles(point, shape, validAngle);
    return angles[0] + (angles[1] - angles[0]) * (percent + 1) / 2;
}

double angleToPercent(const Point& point, double angle, const Shape& shape) {
    double* angles = getMinMaxAngles(point, shape, angle);
    return 2 * (angle - angles[0]) / (angles[1] - angles[0]) - 1;
}

bool rayIntersects(const Point& point, double angle, const Shape& shape) {
    Point curPoint = point;
    Point unit = Point::unit(angle);
    double lastVal = INFINITY;
    for (int i = 0; i < MAX_ITER; ++i) {
        double dist = shape.distance(curPoint);
        if (dist <= 0)
            return true;
        if (dist > lastVal)
            return false;
        lastVal = dist;
        if (dist > large_step_dist)
            curPoint = curPoint + unit * large_step;
        else
            curPoint = curPoint + unit * step;
    }
    return false;
}

double getTangent(const Point& point, const Shape& shape) {
    double dist = shape.distance(point);
    double minAngle = 0;
    double minDeriv = shape.distance(point + Point(std::cos(minAngle), std::sin(minAngle)) * dx) / dx - dist / dx;

    double maxAngle = M_PI;
    double maxDeriv = shape.distance(point + Point(std::cos(maxAngle), std::sin(maxAngle)) * dx) / dx - dist / dx;

    for (int i = 0; i < NUM_ITER; ++i) {
        double midAngle = (minAngle + maxAngle) / 2;
        double midDeriv = shape.distance(point + Point(std::cos(midAngle), std::sin(midAngle)) * dx) / dx - dist / dx;
        if ((midDeriv > 0 && maxDeriv > minDeriv) || (midDeriv < 0 && maxDeriv < minDeriv)) {
            maxAngle = midAngle;
            maxDeriv = midDeriv;
        }
        else {
            minAngle = midAngle;
            minDeriv = midDeriv;
        }
    }

    return (minAngle + maxAngle) / 2;
}

Point getCollisionPoint(const Point& p1, const Point& p2, const Shape& shape) {
    double minDist = shape.distance(p1);
    Point minPoint = p1;
    double maxDist = shape.distance(p2);
    Point maxPoint = p2;

    for (int i = 0; i < NUM_ITER; ++i) {
        Point midPoint = (minPoint + maxPoint) * 0.5;
        double midDist = shape.distance(midPoint);

        if ((midDist > 0 && maxDist > minDist) || (midDist < 0 && maxDist < minDist)) {
            maxPoint = midPoint;
            maxDist = midDist;
        }
        else {
            minPoint = midPoint;
            minDist = midDist;
        }
    }

    return (minPoint + maxPoint) * 0.5;
}

void collideWithShape(const Point& point, double angle, const Shape& shape, Point& outMidpoint, Point& outFinalPoint, double& outNewAngle) {
    Point unit(std::cos(angle), std::sin(angle));
    Point curPoint = point;
    for (int i = 0; i < MAX_ITER; ++i) {
        double dist = shape.distance(curPoint);
        if (dist < 0) {
            outMidpoint = getCollisionPoint(curPoint - unit * step, curPoint, shape);
            break;
        }
        curPoint = curPoint + unit * step;
    }
    double tangentAngle = getTangent(outMidpoint, shape);
    outNewAngle = 2 * tangentAngle - angle;
    double dist = std::sqrt(std::pow(point.x - outMidpoint.x, 2) + std::pow(point.y - outMidpoint.y, 2));
    outFinalPoint = outMidpoint + Point(std::cos(outNewAngle), std::sin(outNewAngle)) * dist;
    // Flip new angle to use as valid angle on next iteration
    outNewAngle += M_PI;
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
            double curPercent = angleToPercent(curPoint, nextAngle, shape);
            curAngle = percentToAngle(curPoint, -curPercent, shape, nextAngle);
        }
    }
}

void runFromPercent(const Point& startPoint, double percent, const Point& validPoint, int n, const Shape& shape, std::vector<Point>& points) {
    double validAngle = std::atan2(validPoint.y - startPoint.y, validPoint.x - startPoint.x);
    double angle = percentToAngle(startPoint, percent, shape, validAngle);
    runIterations(startPoint, angle, n, shape, points);
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

    // Very cool shape
    NEllipse shape({ Point(-1, 1), Point(1, 1), Point(-1, -1), Point(1, -1) }, 4.001 + sqrt(2) * 2);
    Point startPoint(-2, 2);
    double startAngle = .9;


    /*Rectangle shape(Point(-1, 0), Point(1, -1));
    double t = 1;
    
    Point startPoint = Point::unit(t);
    double startAngle = 3*M_PI/2;*/
    bool startAngleIsPercent = true; // If true, startAngle is a number where -1 is furthest left, 1 is furthest right
                                      //, else it's an angle
    // If startAngleIsPercent, you need a valid point to calculate the angle from
    // This point should either be in/on the shape or at least the ray from the startPoint to it should intersect the shape
    Point validPoint = Point(0, 0);

    


    // Create the calculation thread
    std::vector<Point> points;
    std::thread calculationThread;
    calculationThread = std::thread([&] {
		if (startAngleIsPercent)
			runFromPercent(startPoint, startAngle, validPoint, 100000, shape, points);
		else
			runIterations(startPoint, startAngle, 100000, shape, points);
	});
    



    // Create the window

    sf::RenderWindow window(sf::VideoMode(w, h), "SFML Drawing");
    int numDrawn = 1;

    // Pixel array for the shape
    sf::Texture texture;
    texture.create(w, h);
    bool recalculatePixels = true;
    // Main loop
    while (window.isOpen()) {
        // Event handling
        sf::Event event;
        if(window.pollEvent(event)) {
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


        // Draw the sample shape by coloring each pixel based on how close the distance function is to 0
        if (recalculatePixels) {
            recalculatePixels = false;
            static sf::Uint8* pixels = new sf::Uint8[w * h * 4];
            for (int x = 0; x < w; ++x) {
                for (int y = 0; y < h; ++y) {
                    double dist = shape.distance(Point((x - w / 2) / scalingFactor, (h / 2 - y) / scalingFactor));
                    if (std::fabs(dist) < maxDist) {
                        double brightness = 1 - (std::fabs(dist) / maxDist);
                        pixels[(x + y * w) * 4] = brightness * 255;
                        pixels[(x + y * w) * 4 + 1] = brightness * 255;
                        pixels[(x + y * w) * 4 + 2] = brightness * 255;
                        pixels[(x + y * w) * 4 + 3] = 255;
                    }
                    else {
                        pixels[(x + y * w) * 4] = 0;
                        pixels[(x + y * w) * 4 + 1] = 0;
                        pixels[(x + y * w) * 4 + 2] = 0;
                        pixels[(x + y * w) * 4 + 3] = 255;
                    }
                }
            }
            texture.update(pixels);
        }
        
        
        sf::Sprite sprite(texture);
        window.draw(sprite);


        // Check if points has more points than numDrawn
        if (points.size() > numDrawn + 1) {
            numDrawn += 2;
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
