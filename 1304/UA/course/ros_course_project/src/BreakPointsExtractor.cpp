#include <cstddef>
#include <cmath>
#include <tuple>
#include <ros/ros.h>
#include "BreakPointsExtractor.h"

using Point2D = BreakPointsExtractor::Point2D;

std::vector<BreakPointsExtractor::Mask>
BreakPointsExtractor::divideIntoSegmentsWithFiltering(const std::vector<Point2D>& points) const
{
    std::vector<Mask> masks(1);
    masks[0].push_back(0);

    size_t previous = 0;
    size_t current = 0;

    while (++current < points.size())
    {
        float distance = GeomUtils::distance(points[current], points[current - 1]);
        if (distance > m_globalDistance)
        {
            if (masks.back().back() != current - 1)
            {
                distance = GeomUtils::distance(points[masks.back().back()], points[current - 1]);
                if (distance > m_localDistance / 5.f)
                {
                    masks.back().push_back(current - 1);
                }
                else
                {
                    masks.back().back() = current - 1;
                }
            }
            masks.emplace_back();
            masks.back().push_back(current);
            previous = current;
        }
        else
        {
            distance = GeomUtils::distance(points[current], points[previous]);

            if (distance > m_localDistance)
            {
                masks.back().push_back(current);
                previous = current;
            }
        }
    }

    return masks;
}

BreakPointsExtractor::LineType
BreakPointsExtractor::calculateLineType(const Point2D& pointA, const Point2D& pointB) const
{
    float dy = pointA.y - pointB.y;
    float dx = pointA.x - pointB.x;
    return calculateAngles(dy, dx);
}

BreakPointsExtractor::LineType
BreakPointsExtractor::calculateAngles(float dy, float dx, const float MAXIMUM_K) const
{
    float k;
    if (dx == 0.f)
    {
        k = MAXIMUM_K;
    }
    else
    {
        k = dy / dx;
    }

    float angle = std::abs(std::atan(k));


    if (angle > M_PI_2 * m_threshold)
    {
        return LineType::VERTICAL;
    }
    else
    {
        return LineType::HORIZONTAL;
    }
}


std::vector<BreakPointsExtractor::LineInfo>
BreakPointsExtractor::extractLines(const std::vector<Point2D>& points, const Mask& mask) const
{
    std::vector<LineInfo> lines;

    if (mask.size() > 1)
    {
        LineType previousLineType = calculateLineType(points[mask[0]], points[mask[1]]);
        size_t lineBegin = 0;
        size_t current = 1;

        while (++current < mask.size())
        {
            LineType lineType = calculateLineType(points[mask[current]], points[mask[current - 1]]);

            if (lineType != previousLineType)
            {
                size_t currentLineBegin = mask[lineBegin];
                size_t currentLineEnd = mask[current - 1];

                float mean = 0.f;
                if (previousLineType == LineType::VERTICAL)
                {
                    for (size_t i = currentLineBegin; i <= currentLineEnd; ++i)
                    {
                        mean += points[i].x;
                    }
                }
                else
                {
                    for (size_t i = currentLineBegin; i <= currentLineEnd; ++i)
                    {
                        mean += points[i].y;
                    }
                }
                mean /= currentLineEnd - currentLineBegin + 1;

                lines.emplace_back(previousLineType, currentLineBegin, currentLineEnd, mean);
                previousLineType = lineType;
                lineBegin = current - 1;
            }
        }
        size_t currentLineBegin = mask[lineBegin];
        size_t currentLineEnd = mask[current - 1];
        float mean = 0.f;
        if (previousLineType == LineType::VERTICAL)
        {
            for (size_t i = currentLineBegin; i <= currentLineEnd; ++i)
            {
                mean += points[i].x;
            }
        }
        else
        {
            for (size_t i = currentLineBegin; i <= currentLineEnd; ++i)
            {
                mean += points[i].y;
            }
        }
        mean /= currentLineEnd - currentLineBegin + 1;
        lines.emplace_back(previousLineType, mask[lineBegin], mask[current - 1], mean);
    }

    return lines;
}


std::vector<BreakPointsExtractor::Point2D>
BreakPointsExtractor::operator()(const std::vector<Point2D>& points) const
{
    auto masks = divideIntoSegmentsWithFiltering(points);

    std::vector<Point2D> result;
    for (const auto& mask : masks)
    {
        auto lines = extractLines(points, mask);

        if (lines.empty())
        {
            result.push_back(points[mask[0]]);
        }
        else
        {
            result.push_back(points[mask[0]]);

            size_t currentLine = 0;
            while (++currentLine < lines.size())
            {
                LineType currentLineType;
                size_t currentLineBegin, currentLineEnd;
                size_t previousLineBegin, previousLineEnd;
                float currentMean, previousMean;
                std::tie(currentLineType, currentLineBegin, currentLineEnd, currentMean) = lines[currentLine];
                std::tie(std::ignore, previousLineBegin, previousLineEnd, previousMean) = lines[currentLine - 1];

                float x = 0.f;
                float y = 0.f;
                if (currentLineType == LineType::VERTICAL)
                {
                    result.emplace_back(currentMean, previousMean);
                }
                else
                {
                    result.emplace_back(previousMean, currentMean);
                }
            }

            result.push_back(points[mask.back()]);

            if (std::get<0>(lines.front()) == LineType::VERTICAL)
            {
                result[result.size() - lines.size() - 1].x = std::get<3>(lines.front());
            }
            else
            {
                result[result.size() - lines.size() - 1].y = std::get<3>(lines.front());
            }

            if (std::get<0>(lines.back()) == LineType::VERTICAL)
            {
                result.back().x = std::get<3>(lines.back());
            }
            else
            {
                result.back().y = std::get<3>(lines.back());
            }
        }
    }

    if (GeomUtils::distance(points[0], points.back()) < m_localDistance)
    {
        result.pop_back();
        result.erase(result.begin());
    }


    std::vector<Point2D> res;
    size_t base = result.size() - 1;
    for (size_t i = 0; i < result.size(); ++i)
    {
        size_t first = (base + i) % result.size();
        size_t second = (base + i + 1) % result.size();
        size_t third = (base + i + 2) % result.size();

        float dx1 = result[second].x - result[first].x;
        float dy1 = result[second].y - result[first].y;

        float dx2 = result[third].x - result[second].x;
        float dy2 = result[third].y - result[second].y;

        if (std::abs(std::abs(std::atan2(dy1, dx1)) -
                     std::abs(std::atan2(dy2, dx2))) > M_PI * m_lineAngleThreshold)
        {
            res.push_back(result[second]);
        }
    }

    return res;
}
