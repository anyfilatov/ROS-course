#include "LaserScanProcessor.h"


std::vector<LaserScanProcessor::Point2D> LaserScanProcessor::calculatePoints(const LaserScan& laserScan) const
{
    const auto& ranges = laserScan.ranges;

    std::vector<Point2D> points;
    for (size_t index = 0; index < ranges.size(); ++index)
    {
        float alpha = laserScan.angle_max - index * laserScan.angle_increment;
        points.emplace_back(GeomUtils::calculatePoint(ranges[index], -alpha));
    }
    return points;
}

std::vector<LaserScanProcessor::Segment>
LaserScanProcessor::calculateCrumbSegmentsImpl(const std::vector<Point2D>& breakPoints,
                                               const std::vector<size_t>& stack) const
{
    std::vector<std::pair<Point2D, Point2D>> crumbPoints;

    size_t previousPoint = stack.size() - 1;
    size_t currentPoint = 0;
    for (auto& point : stack)
    {
        ROS_INFO("Index i %zu", point);
    }
    do
    {
        size_t previousPointIndex = stack[previousPoint];
        size_t currentPointIndex = stack[currentPoint];
        size_t distanse = (size_t) ((previousPointIndex < currentPointIndex) ?
                                    std::abs(currentPointIndex - previousPointIndex) :
                                    std::abs(currentPointIndex + (breakPoints.size() - previousPointIndex)));
        ROS_DEBUG("DISTANCE i %zu %zu %zu %zu", breakPoints.size(), previousPointIndex, currentPointIndex, distanse);
        if (distanse != 1)
        {
            const Point2D& pointA = breakPoints[currentPointIndex];
            const Point2D& pointB = breakPoints[previousPointIndex];
            crumbPoints.emplace_back(pointA, pointB);
        }
        previousPoint = currentPoint;
        currentPoint += 1;
    }
    while (currentPoint < stack.size());

    return crumbPoints;
}

std::vector<LaserScanProcessor::Segment>
LaserScanProcessor::calculateCrumbSegments(const std::vector<Point2D>& breakPoints, size_t closestPointIndex) const
{
    Point2D center = {0, 0};
    std::vector<size_t> stack = {closestPointIndex,
                                 (closestPointIndex == breakPoints.size() - 1) ? 0 : closestPointIndex + 1};


    for (size_t i = 2; i < breakPoints.size() + 2; ++i)
    {
        size_t index = (closestPointIndex + i) % breakPoints.size();
        const Point2D& pointA = breakPoints[stack[stack.size() - 2]];
        const Point2D& pointB = breakPoints[stack[stack.size() - 1]];
        const Point2D& pointC = breakPoints[index];

        if (cross_product(pointA, pointB, pointC) >= 0.f)
        {
            stack.emplace_back(index);
        }
        else
        {
            if (cross_product(pointB, pointC, center) >= 0.f)
            {
                for (size_t j = stack.size() - 3; j != std::numeric_limits<size_t>::max(); --j)
                {
                    const Point2D& pointNewA = breakPoints[stack[j]];
                    if (cross_product(pointNewA, pointB, pointC) >= 0.f &&
                        cross_product(pointNewA, pointB, center) >= 0.f)
                    {
                        std::swap(stack[stack.size() - 1], stack[j + 1]);
                        stack.resize(j + 2);
                        stack.emplace_back(index);
                        break;
                    }
                }
            }
        }
    }
    if (stack[1] == stack.back())
    {
        stack.pop_back();
    }
    if (stack[0] == stack.back())
    {
        stack.pop_back();
    }
    if (stack[0] == stack[stack.size() - 2])
    {
        stack.pop_back();
        stack.pop_back();
    }

    return calculateCrumbSegmentsImpl(breakPoints, stack);
}

class Line
{
    using Point2D = LaserScanProcessor::Point2D;
public:
    static constexpr float TOLERANCE = 0.1;

    Line(const Point2D& pointA, const Point2D& pointB)
    {
        float dx = pointA.x - pointB.x;
        float dy = pointA.y - pointB.y;

        if (dx == 0.f)
        {
            k = std::numeric_limits<float>::max();
            b = (pointA.x + pointB.x) / 2.f;
        }
        else
        {
            k = dy / dx;

            if (std::abs(k) > 10.f)
            {
                k = std::numeric_limits<float>::max();
                b = (pointA.x + pointB.x) / 2.f;
            }
            else
            {
                b = pointB.y - k * pointB.x;
            }
        }
    }

    bool operator==(const Line& line) const
    {
        return (std::abs(k - line.k) < TOLERANCE) && (std::abs(b - line.b) < TOLERANCE);
    }

    float k;
    float b;
};

std::pair<std::vector<LaserScanProcessor::Point2D>, size_t>
LaserScanProcessor::calculateBreakPointIndexes(const sensor_msgs::LaserScan& laserScan) const
{
    const auto& ranges = laserScan.ranges;

    std::vector<Point2D> points = {
            GeomUtils::calculatePoint(ranges[0], laserScan.angle_min),
            GeomUtils::calculatePoint(ranges[1], laserScan.angle_min + laserScan.angle_increment)};
    std::vector<size_t> indexes = {0, 1};

    Line previousLine{points[0], points[1]};
    ROS_DEBUG("Line k: %f b: %f", previousLine.k, previousLine.b);

    for (size_t i = 2; i < ranges.size(); ++i)
    {
        Point2D point = GeomUtils::calculatePoint(ranges[i], laserScan.angle_min + i * laserScan.angle_increment);
        Line nextLine{points[points.size() - 2], point};
        ROS_DEBUG("Line k: %f z: %f", nextLine.k, nextLine.b);
        if (std::abs(GeomUtils::cross_product(points[points.size() - 2],
                                              points[points.size() - 1],
                                              point)) < 0.01f || previousLine == nextLine)
        {
            points.back() = point;
            indexes.back() = i;
        }
        else
        {
            points.push_back(point);
            indexes.push_back(i);
            previousLine = nextLine;
        }
    }
    if (std::abs(GeomUtils::cross_product(points[points.size() - 1],
                                          points[0],
                                          points[1])) < 0.01f ||
        Line(points[points.size() - 1], points[0]) == Line(points[0], points[1]))
    {
        points.front() = points.back();
        points.pop_back();
        indexes.front() = indexes.back();
        indexes.pop_back();
    }

    size_t closestPointIndex = 0;
    float closestPointDistance = ranges[indexes[0]];
    for (size_t i = 1; i < indexes.size(); ++i)
    {
        size_t index = indexes[i];
        if (ranges[index] < closestPointDistance)
        {
            closestPointIndex = i;
            closestPointDistance = ranges[index];
        }
    }

    return std::make_pair(points, closestPointIndex);
}

std::vector<LaserScanProcessor::Segment>
LaserScanProcessor::extractCrumbsSegments(const std::vector<Point2D>& breakPoints) const
{


    auto minIterator = std::min_element(breakPoints.begin(), breakPoints.end(),
                                        [](const Point2D& pointA, const Point2D& pointB)
                                        {
                                            return GeomUtils::norm(pointA) < GeomUtils::norm(pointB);
                                        });

    size_t closestPointIndex = (size_t) std::distance(breakPoints.begin(), minIterator);

    return calculateCrumbSegments(breakPoints, closestPointIndex);
}