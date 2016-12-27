#include <cmath>
#include "GeomUtils.h"

namespace GeomUtils
{

float distance(const Point2D& point_a, const Point2D& point_b)
{
    return std::sqrt(
            std::pow(point_a.x - point_b.x, 2.0f) +
            std::pow(point_a.y - point_b.y, 2.0f));
}

float norm(const Point2D& point_a)
{
    return std::sqrt(
            std::pow(point_a.x, 2.0f) +
            std::pow(point_a.y, 2.0f));
}

float cross_product(const Point2D& point_a, const Point2D& point_b, const Point2D& point_c)
{
    return (point_c.x - point_a.x) * (point_b.y - point_a.y) -
           (point_c.y - point_a.y) * (point_b.x - point_a.x);
}

Point2D calculatePoint(float r, float alpha)
{
    return {(float) (r * std::sin(alpha)), (float) (r * std::cos(alpha))};
}

Point2D rotate(const Point2D& point, float angle)
{
    float x = point.x;
    float y = point.y;

    return {x * std::cos(angle) - y * std::sin(angle),
            x * std::sin(angle) + y * std::cos(angle)};
}

void rotate(std::vector<Point2D>& points, float angle)
{
    for (auto& point : points)
    {
        point = rotate(point, angle);
    }
}

void rotate(std::vector<Segment>& segments, float angle)
{
    for (auto& segment : segments)
    {
        segment.first = rotate(segment.first, angle);
        segment.second = rotate(segment.second, angle);
    }
}

Point2D move(const Point2D& point, float x, float y)
{

    return {point.x + x, point.y + y};
}

void move(std::vector<Point2D>& points, float x, float y)
{
    for (auto& point :points)
    {
        point = move(point, x, y);
    }
}

void move(std::vector<Segment>& segments, float x, float y)
{
    for (auto& segment : segments)
    {
        segment.first = move(segment.first, x, y);
        segment.second = move(segment.second, x, y);
    }
}

}
