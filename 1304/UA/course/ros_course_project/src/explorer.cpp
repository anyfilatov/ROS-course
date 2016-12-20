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

enum class State
{
    CHOOSING, ROTATING, MOVING, STEPPING, AVOID_WALL
};


int main(int argc, char** argv)
{
    using GeomUtils::Point2D;

    ros::init(argc, argv, "explorer");

    ros::NodeHandle nodeHandle;

    RvizVisualizer visualizer(nodeHandle, "explorer");
    LaserScanProcessor scanProcessor(&visualizer);
    RobotController explorer(nodeHandle);
    BreakPointsExtractor breakPointsExtractor;

    ros::Rate r(60);

    State state = State::CHOOSING;
    Point2D currentPoint;
    Point2D neWcurrentPoint;
    std::default_random_engine re;

    size_t step = 0;
    size_t shadowStep = 0;
    while (ros::ok())
    {
        const Point2D& currentPosition = explorer.getCurrentPosition();
        ROS_INFO("Current position{x: %f y: %f theta: %f diff: %f dist: %f state: %i}",
                 currentPosition.x,
                 currentPosition.y,
                 explorer.getCurrentAngle(),
                 explorer.getAngleToTarget(currentPoint),
                 explorer.getDistanceToTarget(currentPoint),
                 state);


        if (explorer.haveLaseScan() &&
            explorer.getLaserScan().ranges[explorer.getLaserScan().ranges.size() / 2] < 0.5)
        {
            state = State::AVOID_WALL;
        }

        if (explorer.haveNewLaserScan())
        {
            if (step % 10 == 0)
            {
                auto points = scanProcessor.calculatePoints(explorer.getLaserScan());
                GeomUtils::rotate(points, -explorer.getCurrentAngle());

                visualizer.publishMarker("base_mark", points);
                for (const auto& point : points)
                {
                    ROS_INFO("Base point x: %f y: %f", point.x, point.y);
                }


                auto breakPoints = breakPointsExtractor(points);
                visualizer.publishMarker("sorted_markers", breakPoints, 1);
                visualizer.publishMarker("sorted_markers_points", breakPoints, 0);
                for (auto& point : breakPoints)
                {
                    ROS_INFO("Break point x: %f y: %f", point.x, point.y);
                }

                const auto crumbSegments = scanProcessor.extractCrumbsSegments(breakPoints);
                std::vector<Point2D> crumbPoints;
                for (const auto& segment : crumbSegments)
                {
                    crumbPoints.emplace_back((segment.first.x + segment.second.x) / 2.f,
                                             (segment.first.y + segment.second.y) / 2.f);
                    ROS_INFO("Crumb point x: %f y: %f", crumbPoints.front().x, crumbPoints.front().y);
                }
                visualizer.publishMarker("cumber_markers", crumbPoints);
                GeomUtils::rotate(crumbPoints, explorer.getCurrentAngle());

                std::uniform_int_distribution<size_t> rd(0, (int) crumbPoints.size());
                currentPoint = crumbPoints[rd(re)];

                neWcurrentPoint.x += currentPosition.x;
                neWcurrentPoint.y += currentPosition.y;

                std::vector<Point2D> target;
                target.push_back(currentPoint);
                visualizer.publishMarker("target", target, 0);


                ROS_INFO("Done");
            }
        }

        switch (state)
        {
            case State::CHOOSING:
            {
                if (explorer.haveNewLaserScan())
                {
                    currentPoint = neWcurrentPoint;
                    state = State::ROTATING;
                }
                break;
            }


            case State::ROTATING:
            {
                float angle = explorer.getAngleToTarget(currentPoint);
                if (std::abs(angle) > M_PI * 0.05)
                {
                    if (angle < 0)
                    {
                        explorer.rotate(-0.4f);
                    }
                    else
                    {
                        explorer.rotate(0.4f);
                    }
                }
                else
                {
                    state = State::MOVING;
                }
                break;
            }

            case State::MOVING:
            {
                if (std::abs(explorer.getAngleToTarget(currentPoint)) > M_PI * 0.05)
                {
                    state = State::ROTATING;
                }
                else if (explorer.getDistanceToTarget(currentPoint) > 0.1)
                {
                    explorer.moveForward(0.4);
                }
                else
                {
                    state = State::STEPPING;
                    shadowStep = 0;
                }
                break;
            }

            case State::STEPPING:
            {
                if (++shadowStep < 10)
                {
                    explorer.moveForward(0.1);
                }
                else
                {
                    state = State::CHOOSING;
                }
                break;
            }

            case State::AVOID_WALL:
            {
                if (explorer.haveLaseScan() &&
                    explorer.getLaserScan().ranges[explorer.getLaserScan().ranges.size() / 2] < 0.5)
                {
                    explorer.moveForward(-0.2f);
                }
                else
                {
                    explorer.moveForward(0.0);
                    state = State::CHOOSING;
                }
            }
        }


        ros::spinOnce();
        ++step;
        r.sleep();
    }

    return 0;
}
