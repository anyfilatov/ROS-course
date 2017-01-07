#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <RobotController.h>
#include <RvizVisualizer.h>
#include <LaserScanProcessor.h>
#include <BreakPointsExtractor.h>

class FuzzyPointComparator
{
public:
    FuzzyPointComparator(float tolerance) : TOLERANCE(tolerance)
    { }

    bool operator()(const GeomUtils::Point2D& p1, GeomUtils::Point2D p2)
    {
        if (GeomUtils::distance(p1, p2) < TOLERANCE)
        {
            return false;
        }

        if (p1.x < p2.x) return true;
        if (p2.x > p1.x) return false;

        if (p1.y > p2.y) return true;
        if (p2.y > p1.y) return false;

        return false;
    }

private:
    const float TOLERANCE = 1.f;
};

std::map<GeomUtils::Point2D, int, FuzzyPointComparator> CRUMBS_POINTS(FuzzyPointComparator(1.0f));

std::pair<GeomUtils::Point2D, GeomUtils::Point2D>
getNextTarget(const std::vector<GeomUtils::Segment>& crumbSegments,
              GeomUtils::Point2D robotPosition,
              float distance = 0.2f,
              RvizVisualizer* visualizer = nullptr)
{
    using GeomUtils::Point2D;
    std::vector<Point2D> crumbPoints;
    for (const auto& segment : crumbSegments)
    {
        crumbPoints.emplace_back((segment.first.x + segment.second.x) / 2.f,
                                 (segment.first.y + segment.second.y) / 2.f);
        ROS_DEBUG("Crumb point x: %f y: %f", crumbPoints.front().x, crumbPoints.front().y);
    }
    if (visualizer)
    {
        visualizer->publishMarker("cumber_markers", crumbPoints);
    }

    size_t offset = std::numeric_limits<size_t>::max();
    float maxDistance = std::numeric_limits<size_t>::min();
    for (size_t i = 0; i < crumbPoints.size(); ++i)
    {
        Point2D point = crumbPoints[i];
        std::swap(point.x, point.y);
        point += robotPosition;
        const auto& iterator = CRUMBS_POINTS.find(point);
        if (iterator == CRUMBS_POINTS.end())
        {
            float norm = GeomUtils::norm(point);
            if (norm > maxDistance)
            {
                maxDistance = norm;
                offset = i;
            }
        }
    }

    if (offset == std::numeric_limits<size_t>::max())
    {
        for (size_t i = 0; i < crumbPoints.size(); ++i)
        {
            Point2D point = crumbPoints[i];
            std::swap(point.x, point.y);
            point += robotPosition;
            const auto& iterator = CRUMBS_POINTS.find(point);
            if (iterator != CRUMBS_POINTS.end() && (*iterator).second != 2)
            {
                float norm = GeomUtils::norm(point);
                if (norm > maxDistance)
                {
                    maxDistance = norm;
                    offset = i;
                }
            }
        }
    }

    if (offset == std::numeric_limits<size_t>::max())
    {
        throw std::runtime_error("NO EXIT");
    }

    Point2D currentPoint = crumbPoints[offset];
    auto currentSegment = crumbSegments[offset];
    Point2D nextPoint;

    Point2D norm{-(currentSegment.first.y - currentSegment.second.y),
                 currentSegment.first.x - currentSegment.second.x};
    norm = norm.norm() * distance;
    ROS_INFO("NORM point x: %f y: %f", norm.x, norm.y);

    float zeroCP = GeomUtils::cross_product(currentSegment.first, currentSegment.second, Point2D{0, 0});
    if (GeomUtils::cross_product(currentSegment.first, currentSegment.second, currentPoint + norm) * zeroCP <= 0)
    {
        nextPoint = currentPoint + norm;
    }
    else
    {
        nextPoint = currentPoint + norm * -1.f;
    }

    if (visualizer)
    {
        std::vector<Point2D> target{currentPoint, nextPoint};
        visualizer->publishMarker("target", target, 0);
    }
    return {currentPoint, nextPoint};
}

enum class RobotState
{
    ANALYZE, MOVEMENT, FINISHED
};

bool checkWinCondition(const sensor_msgs::LaserScan& laserScan)
{
    const auto& ranges = laserScan.ranges;
    for (size_t i = 0; i < ranges.size(); ++i)
    {
        if (ranges[i] > 40.0 &&
            ranges[(i - 200) % ranges.size()] > 40.0 &&
            ranges[(i + 200) % ranges.size()] > 40.0)
        {
            return true;
        }
    }
    return false;
}

int main(int argc, char** argv)
{
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    using GeomUtils::Point2D;

    ros::init(argc, argv, "explorer");

    ros::NodeHandle nodeHandle;

    RvizVisualizer visualizer(nodeHandle, "explorer");
    LaserScanProcessor scanProcessor(&visualizer);
    RobotController explorer(nodeHandle);
    BreakPointsExtractor breakPointsExtractor;

    ros::Rate r(60);

    Point2D currentPoint;

    size_t step = 0;
    RobotState state = RobotState::ANALYZE;
    //
    std::vector<Point2D> points;
    std::vector<Point2D> breakPoints;
    bool newDataAvailable = false;
    //
    std::pair<Point2D, Point2D> pathPlan;
    bool isFirstStep = true;
    Point2D& currentGoal = pathPlan.first;
    //

    while (ros::ok())
    {
        const Point2D& currentPosition = explorer.getCurrentPosition();
        ROS_INFO("Current position{x: %f y: %f theta: %f diff: %f dist: %f}",
                 currentPosition.x,
                 currentPosition.y,
                 explorer.getCurrentAngle(),
                 explorer.getAngleToTarget(currentPoint),
                 explorer.getDistanceToTarget(currentPoint));

        if (explorer.haveNewLaserScan())
        {
            if (step % 10 == 0)
            {
                points = scanProcessor.calculatePoints(explorer.getLaserScan());

                if (checkWinCondition(explorer.getLaserScan()))
                {
                    break;
                }

                GeomUtils::rotate(points, -explorer.getCurrentAngle());
                visualizer.publishMarker("base_mark", points);
                for (const auto& point : points)
                {
                    ROS_DEBUG("Base point x: %f y: %f", point.x, point.y);
                }

                breakPoints = breakPointsExtractor(points);
                visualizer.publishMarker("sorted_markers", breakPoints, 1);
                visualizer.publishMarker("sorted_markers_points", breakPoints, 0);
                for (auto& point : breakPoints)
                {
                    ROS_DEBUG("Break point x: %f y: %f", point.x, point.y);
                }
                newDataAvailable = true;
            }
        }

        switch (state)
        {
            case RobotState::ANALYZE:
            {
                if (newDataAvailable)
                {
                    newDataAvailable = false;
                    const auto crumbSegments = scanProcessor.extractCrumbsSegments(breakPoints);

                    pathPlan = getNextTarget(crumbSegments, explorer.getCurrentPosition(), 1.f, &visualizer);
                    currentGoal = pathPlan.first;
                    isFirstStep = true;

                    std::swap(pathPlan.first.x, pathPlan.first.y);
                    std::swap(pathPlan.second.x, pathPlan.second.y);
                    pathPlan.first += explorer.getCurrentPosition();
                    pathPlan.second += explorer.getCurrentPosition();

                    std::vector<Point2D> target{pathPlan.first, pathPlan.second};
                    visualizer.publishMarker("target2", target, 0);

                    state = RobotState::MOVEMENT;

                }
                break;
            }
            case RobotState::MOVEMENT:
            {
                ROS_INFO("DISTANCE %f ANGLE %f", GeomUtils::distance(explorer.getCurrentPosition(), currentGoal),
                         explorer.getAngleToTarget(currentGoal));
                if (GeomUtils::distance(explorer.getCurrentPosition(), currentGoal) < 0.1f)
                {
                    if (isFirstStep)
                    {
                        const auto& crumbIter = CRUMBS_POINTS.find(explorer.getCurrentPosition());
                        if (crumbIter == CRUMBS_POINTS.end())
                        {
                            CRUMBS_POINTS[explorer.getCurrentPosition()] = 1;
                        }
                        else
                        {
                            CRUMBS_POINTS[explorer.getCurrentPosition()] = crumbIter->second + 1;
                        }
                        isFirstStep = false;
                        currentGoal = pathPlan.second;
                    }
                    else
                    {
                        state = RobotState::ANALYZE;
                    }
                }
                else if (std::abs(explorer.getAngleToTarget(currentGoal)) < 0.01f)
                {
                    ROS_INFO("%f", std::abs(explorer.getAngleToTarget(currentGoal)));
                    explorer.moveForward(0.4f);
                }
                else if (explorer.getAngleToTarget(currentGoal) < 0.0f)
                {
                    ROS_INFO("%f", std::abs(explorer.getAngleToTarget(currentGoal)));
                    explorer.rotate(-0.15f);
                }
                else
                {
                    ROS_INFO("%f", std::abs(explorer.getAngleToTarget(currentGoal)));
                    explorer.rotate(0.15f);
                }
                break;
            }
            case RobotState::FINISHED:break;
        }


        ROS_INFO("Step done");
        ros::spinOnce();
        ++step;
        r.sleep();
    }

    ROS_INFO("I FOUND EXIT");

    return 0;
}
