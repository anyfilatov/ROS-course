#ifndef COURSE_EXPLORER_BREAKPOINTSEXTRACTOR_H
#define COURSE_EXPLORER_BREAKPOINTSEXTRACTOR_H


#include "GeomUtils.h"

class BreakPointsExtractor
{
public:
    using Point2D = GeomUtils::Point2D;

    BreakPointsExtractor(float localDistance = 0.5f,
                         float globalDistance = 1.0f,
                         float threshold = 0.5f,
                         float lineAngleThreshold = 0.01f) :
            m_localDistance{localDistance},
            m_globalDistance{globalDistance},
            m_threshold{threshold},
            m_lineAngleThreshold{lineAngleThreshold}
    { }

    std::vector<Point2D> operator()(const std::vector<Point2D>& points) const;

private:
    enum class LineType
    {
        VERTICAL,
        HORIZONTAL
    };

    using LineInfo = std::tuple<LineType, size_t, size_t, float>;

    using Mask = std::vector<size_t>;
    std::vector<Mask> divideIntoSegmentsWithFiltering(const std::vector<Point2D>& points) const;
    LineType calculateLineType(const Point2D& pointA, const Point2D& pointB) const;
    LineType calculateAngles(float dy, float dx, const float MAXIMUM_K = 50000.f) const;

    std::vector<BreakPointsExtractor::LineInfo>
    extractLines(const std::vector<Point2D>& points, const Mask& masks) const;

    float m_localDistance;
    float m_globalDistance;
    float m_threshold;
    float m_lineAngleThreshold;
};


#endif //COURSE_EXPLORER_BREAKPOINTSEXTRACTOR_H
