#define _USE_MATH_DEFINES


#include <iostream>
#include <cmath>
#include <SFML/Graphics.hpp>
#include <thread>
#include "transform.hpp"


// Drawing constants
const int pointRadius = 5;
double scalingFactor = 200; // How big to scale from plane to screen
double maxDist = 0.03; // Max distance to the main shape to draw
int numPointsOfShape = 1000; // Number of points on the shape to draw
const double lineOpacity = 0.5;

// Window size
int w = 2000;
int h = 1500;

// Minimum Time between polling for mouse clicks and stuff
// Can be set to 0 probably
double frameUpdateTime = 0.001;
// Number of frames to wait before drawing the next point (if there is a point to draw)
int framesBetweenDraws = 0;

// STARTING CONDITIONS ARE IN THE MAIN FUNCTION

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
    line[0].color = sf::Color(color.r, color.g, color.b, lineOpacity * 255);
    line[1].color = sf::Color(color.r, color.g, color.b, lineOpacity * 255);
    window.draw(line, 2, sf::Lines);
}

Point screenPositionToPoint(double x, double y) {
    return Point(x / scalingFactor - w / (2 * scalingFactor), -y / scalingFactor + h / (2 * scalingFactor));
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


// Example shapes

Point shape1(double noPiT) {
    double t = 2 * M_PI * noPiT;
    // x(t) = cos(2pi*t) + tsin(2pi * t) – pi * t^2 + pi * t and y(t) = -0.4 sin(2pi * t)
    double x = cos(t) + noPiT * sin(t) - M_PI * noPiT * noPiT + M_PI * noPiT;
    double y = -.4 * sin(t);
    return(Point(x, y));
}


Point shape2(double t) {
    return Point(t, .05*sin(t));
}


// Without starting parameters
void runWithDraw(Shape & shape, bool givenStartParameters, Point startPoint, double startAngle, int numToCalculate) {
    std::vector<Point> points;
    std::thread calculationThread;
    if (givenStartParameters) {
        calculationThread = std::thread([&] {
            runIterations(startPoint, startAngle, numToCalculate, shape, points);
        });
    }

    DrawState state = givenStartParameters ? DrawState::DRAWING : DrawState::WAITING_FOR_POINT;
    // Create the window
    sf::RenderWindow window(sf::VideoMode(w, h), "SFML Drawing");
    int numDrawn = 1;

    // Pixel array for the shape
    sf::VertexArray curve;
    bool recalculatePixels = true;

    // Clicked this frame
    bool leftMouseClicked = false;

    int curDrawFrameNumber = 0; // Number of frames since the last point was drawn

    // Main loop
    while (window.isOpen()) {
        // Left mouse stuff
        //leftMouseClicked = !leftMouseDown && sf::Mouse::isButtonPressed(sf::Mouse::Left);
        leftMouseClicked = false;

        // Event handling
        sf::Event event;
        if (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                killCalculation();
                // Note that the thread has likely already been joined in the left click drawing section
                window.close();
            }
            // On resize, edit w and h
            if (event.type == sf::Event::Resized) {
                w = event.size.width;
                h = event.size.height;
                recalculatePixels = true;
            }
            // Check if the up or down arrows are pressed for zooming
            if (event.type == sf::Event::KeyPressed) {
                if (event.key.code == sf::Keyboard::Up) {
                    scalingFactor *= 1.1;
                    recalculatePixels = true;
                }
                if (event.key.code == sf::Keyboard::Down) {
                    scalingFactor /= 1.1;
                    recalculatePixels = true;
                }
            }
            if (event.type == sf::Event::MouseButtonPressed) {
                leftMouseClicked = true;

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

        Point mousePos = screenPositionToPoint(sf::Mouse::getPosition(window).x, sf::Mouse::getPosition(window).y);

        switch (state) {
        case DrawState::WAITING_FOR_POINT:
            if (leftMouseClicked) {
                startPoint = mousePos;
                state = DrawState::WAITING_FOR_ANGLE;
                leftMouseClicked = false;
                // Go into the next state (don't break out of switch)
            }
            // Otherwise, draw the point sligthly transparent
            else {
                drawPoint(window, mousePos, scalingFactor, sf::Color(255, 0, 0, 100));
                break;
            }
        case DrawState::WAITING_FOR_ANGLE:
            // Draw the start point
            drawPoint(window, startPoint, scalingFactor, sf::Color(255, 0, 0, 255));
            if (leftMouseClicked) {
                startAngle = atan2(mousePos.y - startPoint.y, mousePos.x - startPoint.x);
                state = DrawState::DRAWING;
                // Start the calculation thread
                calculationThread = std::thread([&] {
                    runIterations(startPoint, startAngle, numToCalculate, shape, points);
                });
                leftMouseClicked = false;
                // Printout these conditions
                std::cout << "Start Point: (" << startPoint.x << ", " << startPoint.y << ")" << std::endl;
                std::cout << "Start Angle: " << startAngle << std::endl;
                std::cout << std::endl;
            }
            // Otherwise, draw the ray from the start point through the mouse
            else {
                Point endPoint = screenPositionToPoint(sf::Mouse::getPosition(window).x, sf::Mouse::getPosition(window).y);
                Point rayEnd = startPoint + (endPoint - startPoint) * 100;
                drawLine(window, startPoint, rayEnd, scalingFactor, sf::Color(255, 0, 0, 100));
            }
            break;
        case DrawState::DRAWING:
            if (curDrawFrameNumber >= framesBetweenDraws) {
                curDrawFrameNumber = 0;
                // Check if points has more points than numDrawn
                if (points.size() > numDrawn) {
                    numDrawn++;
                    // Draw the points and lines
                    for (int i = 0; i < numDrawn; ++i) {
                        double r = 0, g = 0, b = 0;
                        HSLtoRGB(300 * i / numDrawn, 1, 0.5, r, g, b);
                        sf::Color color(r * 255, g * 255, b * 255);
                        drawPoint(window, points[i], scalingFactor, color);
                        if (i > 0)
                            drawLine(window, points[i - 1], points[i], scalingFactor, color);
                    }
                }
            }
            else curDrawFrameNumber++;
            // Check if mouse clicked to restart
            if (leftMouseClicked) {
                killCalculation();
                calculationThread.join();
                points.clear();
                numDrawn = 1;
                unkillCalculation();
                state = DrawState::WAITING_FOR_POINT;
            }

        }

        window.display();

        // Sleep for frame update time
        std::this_thread::sleep_for(std::chrono::milliseconds((int)(frameUpdateTime * 1000)));
    }
    
}


void printPoints(Shape& shape, Point startPoint, double startAngle, int numToCalculate) {

    std::vector<Point> points;
	runIterations(startPoint, startAngle, numToCalculate, shape, points);
    for (int i = 0; i < points.size(); i++) {
		std::cout << "(" << points[i].x << ", " << points[i].y << "),";
	}
	std::cout << std::endl;

}


int main() {

    // Example usage
    //  Circle shape(1);
    // Create a rectangle
    // Rectangle shape(1,1,0.0);
    // Create an ellipse
    // Ellipse shape(Point(-1, 0), Point(1, 0), 2.5);
    /*
    double r = sqrt(3);
    double theta = 1.4 * M_PI / 4 + 0.1;
    Point startPoint(r * cos(theta), r * sin(theta));
    double startAngle = atan2(-startPoint.y, -startPoint.x);
    */

 

    // Ellipse shape(1, 0.5);
    //// Circle shape(1.2);

    //// Ellipse shape(1, 0.4);

    Point startPoint = Point(1.30309, 2.73449);

    double startAngle = -1.90384;

    ArbitraryShape shape = ArbitraryShape(shape1, true, 0, 1);
    // Whether or not to actually draw
    bool draw = true;
    // Whether or not we have been given start parameters or if we should use what is drawn by the user
    bool givenStartParameters = true;
    int numToCalculate = draw ? 100000 : 300;

    //Segment shape(1);
    //Point startPoint = Point(-sqrt(2)/2, sqrt(2)/2);
    //// Angle from start point to the origin
    //double startAngle = atan2(-startPoint.y, -startPoint.x);
    

    /*if (draw)
        runWithDraw(shape, givenStartParameters, startPoint, startAngle, numToCalculate);
    else
        printPoints(shape, startPoint, startAngle, numToCalculate);*/

    // Array of point vectors, size 100
    std::vector<Point> pointArrays[100];
    double squareHalfRadius = 0.2;

    for (int i = 0; i < 10; i++) {
        for (int j = 0; j < 10; j++) {
            Point starter = startPoint - Point(squareHalfRadius, squareHalfRadius) 
                + Point(2 * squareHalfRadius * (i / 10.0), 2 * squareHalfRadius * (j / 10.0));
            runIterations(starter, startAngle, 10, shape, pointArrays[i * 10 + j]);
		}
    }

    // Loop through arrays and print each square
    for (int i = 0; i < 10; i++) {
        std::cout << "[";
        for (int j = 0; j < 100; j++) {
            Point p = pointArrays[j][2 * i];
            std::cout << "(" << p.x << ", " << p.y << ")";
            if (j < 99) {
				std::cout << ", ";
			}
        }
        std::cout << "]" << std::endl;
    }
    
    return 0;
}
