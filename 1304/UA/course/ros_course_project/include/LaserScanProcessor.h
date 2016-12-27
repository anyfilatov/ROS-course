#ifndef COURSE_EXPLORER_LASERSCANPROCESSOR_H
#define COURSE_EXPLORER_LASERSCANPROCESSOR_H


#include <vector>
#include <sensor_msgs/LaserScan.h>
#include "GeomUtils.h"
#include "RvizVisualizer.h"

class LaserScanProcessor
{
public:
    using Point2D = GeomUtils::Point2D;
    using Segment = GeomUtils::Segment;
    using LaserScan = sensor_msgs::LaserScan;
    using RangesType = sensor_msgs::LaserScan::_ranges_type;

    LaserScanProcessor(RvizVisualizer* visualizer = nullptr) : m_visualizer(visualizer)
    { }

    std::vector<Point2D> calculatePoints(const LaserScan& laserScan) const;

    //std::vector<Segment> extractCrumbsSegments(const sensor_msgs::LaserScan& laserScan) const;
    std::vector<Segment> extractCrumbsSegments(const std::vector<Point2D>& breakPoints) const;

private:
    std::vector<Segment> calculateCrumbSegments(const std::vector<Point2D>& vector,
                                                                    size_t closestPointIndex) const;

    std::vector<Segment>
    calculateCrumbSegmentsImpl(const std::vector<Point2D>& breakPoints, const std::vector<size_t>& stack) const;

    std::pair<std::vector<Point2D>, size_t> calculateBreakPointIndexes(const sensor_msgs::LaserScan& laserScan) const;

    RvizVisualizer* m_visualizer;
};


#endif //COURSE_EXPLORER_LASERSCANPROCESSOR_H
