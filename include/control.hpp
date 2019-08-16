#pragma once

#include <vector>

#include <opencv2/core/core.hpp>

namespace sim
{

struct CircleMatch
{
    cv::Point2f pos;
    float radius;
};

typedef std::vector<std::vector<cv::Point>> CircleContour;

class PlaneControl
{
public:
    PlaneControl(bool debugRender = true);
    ~PlaneControl() = default;

    void resetPredictions();

    int getPlanePositionPrediction(cv::Mat &ballFrame, int planeDistPx);
    double getAveragePredictedRadiusPx();

private:
    bool m_isDebugRenderEnabled;
    std::vector<CircleMatch> m_ballPoints;

    void getRedFilteredFrame(const cv::Mat &frame, cv::Mat &binResultMask);
    int getCircleContours(cv::Mat &frame, CircleContour &outContours);
};

} // namespace sim
