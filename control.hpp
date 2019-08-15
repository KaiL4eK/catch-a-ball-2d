#include <vector>

#include <opencv2/core/core.hpp>

struct CircleMatch {
    cv::Point2f     pos;
    float           radius;
};

class PlaneControl
{
public:

    PlaneControl();
    ~PlaneControl();

    void resetPredictions();

    int getPlanePositionPrediction(cv::Mat &ballFrame, int plabeDistPx);

private:

    cv::Mat getRedCircleFilteredFrame(cv::Mat &frame);
    int getCircles(cv::Mat &bin_balls, std::vector<CircleMatch> &circles);
};
