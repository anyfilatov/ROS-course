#ifndef COURSE_EXPLORER_VIZUALIZATIOR_H
#define COURSE_EXPLORER_VIZUALIZATIOR_H

#include <ros/node_handle.h>
#include <visualization_msgs/Marker.h>
#include <unordered_map>

#include "GeomUtils.h"

class RvizVisualizer
{
    using Point2D = GeomUtils::Point2D;
    using Color = std_msgs::ColorRGBA;

public:

    RvizVisualizer(ros::NodeHandle& nodeHandle, const std::string& name);

    void publishMarker(const std::string& name, const std::vector<Point2D>& points, size_t choise = 0);

private:
    Color getColor(const std::string& name);

    std::string m_name;
    ros::Publisher m_markerPublisher;
    std::unordered_map<std::string, std_msgs::ColorRGBA> m_colorMap;
    std::default_random_engine m_re;
    std::uniform_real_distribution<float> m_rd;
};


#endif //COURSE_EXPLORER_VIZUALIZATIOR_H
