#include "RvizVisualizer.h"

RvizVisualizer::RvizVisualizer(ros::NodeHandle& nodeHandle, const std::string& name)
        : m_name(name),
          m_rd(0.5f, 1.0f)
{
    m_markerPublisher = nodeHandle.advertise<visualization_msgs::Marker>("/marker", 100);
}

void RvizVisualizer::publishMarker(const std::string& name, const std::vector<Point2D>& points, size_t markerType)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/odom";
    marker.header.stamp = ros::Time::now();
    marker.ns = name;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.orientation.w = 1.0;

    marker.id = 0;
    if (markerType == 0)
    {
        marker.type = visualization_msgs::Marker::POINTS;
    }
    else if (markerType == 1)
    {
        marker.type = visualization_msgs::Marker::LINE_STRIP;
    }

    marker.scale.x = 0.5;
    marker.scale.y = 0.5;

    marker.color = getColor(name);

    geometry_msgs::Point markerPoint;
    for (const auto& point : points)
    {
        markerPoint.x = point.x;
        markerPoint.y = point.y;
        marker.points.push_back(markerPoint);
    }
    if (markerType == 0) {
        markerPoint.x = 0;
        markerPoint.y = 0;
        marker.points.push_back(markerPoint);
    }
    if (markerType == 1) {
        marker.points.emplace_back(marker.points.front());
    }
    m_markerPublisher.publish(marker);
}

RvizVisualizer::Color RvizVisualizer::getColor(const std::string& name)
{
    auto it = m_colorMap.find(name);

    if (it != m_colorMap.end())
    {
        return it->second;
    }
    else
    {
        Color newColor;
        newColor.r = m_rd(m_re);
        newColor.g = m_rd(m_re);
        newColor.b = m_rd(m_re);
        newColor.a = 1.0f;
        m_colorMap[name] = newColor;
        return newColor;
    }
}
