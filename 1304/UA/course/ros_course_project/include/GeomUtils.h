#ifndef COURSE_EXPLORER_GEOMUTILS_H
#define COURSE_EXPLORER_GEOMUTILS_H


#include <utility>
#include <vector>

namespace GeomUtils
{

const float EPSILON = 0.05;
const float ANGULAR_SPEED = 0.4f;
const float LINEAR_SPEED = 1.0f;
const float DISTANCE_EPSILON = 1.0f;
const unsigned int FRAME_LIMIT = 20;

struct Point2D
{
    float x;
    float y;

    Point2D() = default;

    Point2D(float x, float y) : x(x), y(y)
    { }
};

using Segment = std::pair<Point2D, Point2D>;

float norm(const Point2D& point_a);

float distance(const Point2D& point_a, const Point2D& point_b);

float cross_product(const Point2D& point_a, const Point2D& point_b, const Point2D& point_c);

Point2D calculatePoint(float r, float alpha);

Point2D rotate(const Point2D& points, float angle);

void rotate(std::vector<Point2D>& points, float angle);

void rotate(std::vector<Segment>& segments, float angle);

Point2D move(const Point2D& point, float x, float y);

void move(std::vector<Point2D>& points, float x, float y);

void move(std::vector<Segment>& segments, float x, float y);

};

#endif //COURSE_EXPLORER_GEOMUTILS_H
