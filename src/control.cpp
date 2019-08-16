#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "control.hpp"

using namespace cv;
using namespace std;
using namespace sim;

#include <Eigen/QR>

const double MIN_BALL_RADIUS_PX = 10;
const double MIN_CIRCLE_ECCENTRICITY = 0.9;
const uint32_t MIN_TRAJECTORY_POINTS = 10;

static void polyfit(const vector<Point2f> &pnts,
                    vector<double> &coeff,
                    int order)
{
    vector<double> yv(pnts.size());

    for (size_t i = 0; i < pnts.size(); i++)
    {
        yv[i] = pnts[i].y;
    }

    Eigen::MatrixXd A(pnts.size(), order + 1);
    Eigen::VectorXd yv_mapped = Eigen::VectorXd::Map(&yv.front(), yv.size());
    Eigen::VectorXd result;

    assert(pnts.size() >= order + 1);

    for (size_t i = 0; i < pnts.size(); i++)
    {
        for (size_t j = 0; j < order + 1; j++)
        {
            A(i, j) = pow(pnts[i].x, j);
        }
    }

    result = A.householderQr().solve(yv_mapped);

    coeff.resize(order + 1);
    for (size_t i = 0; i < order + 1; i++)
    {
        coeff[i] = result[i];
    }
}

PlaneControl::PlaneControl(bool debugRender) : 
    m_isDebugRenderEnabled(debugRender)
{
}

void PlaneControl::resetPredictions()
{
    m_ballPoints.clear();
}

int PlaneControl::getPlanePositionPrediction(Mat &ballFrame, int planeDistPx)
{
    CircleContour contours;
    vector<CircleMatch> circles;
    int resultControlY;

    getCircleContours(ballFrame, contours);

    for (const vector<Point> &contour : contours)
    {
        CircleMatch match;

        minEnclosingCircle(contour,
                           match.pos,
                           match.radius);

        if (match.radius < MIN_BALL_RADIUS_PX)
            continue;

        circles.push_back(match);
    }

    if (circles.size() > 0)
    {
        m_ballPoints.push_back(circles[0].pos);
    }

    /* Approximate previous points to predict future trajectory */
    if (m_ballPoints.size() > MIN_TRAJECTORY_POINTS)
    {
        vector<double> coeffs;
        polyfit(m_ballPoints, coeffs, 2);

        assert(coeffs.size() == 3);

        if (m_isDebugRenderEnabled)
        {
            for (int x = 0; x < ballFrame.size().width; x++)
            {
                int y = coeffs[0] + x * coeffs[1] + x * x * coeffs[2];

                circle(ballFrame, Point(x, y), 1,
                       Scalar(0, 0, 128), FILLED);
            }
        }

        resultControlY = coeffs[0] +
                         planeDistPx * coeffs[1] +
                         planeDistPx * planeDistPx * coeffs[2];
    }

    /* Render detected centers */
    if (m_isDebugRenderEnabled)
    {
        for (auto pnt_mtc : m_ballPoints)
        {
            circle(ballFrame,
                   pnt_mtc,
                   2,
                   Scalar(255, 0, 128),
                   FILLED);
        }
    }

    return resultControlY;
}

void PlaneControl::getRedFilteredFrame(const Mat &frame, Mat &binResultMask)
{
    /* In real situation GaussianBlur() is required */
    Mat hsv;
    cvtColor(frame, hsv, COLOR_BGR2HSV);

    Mat lowerRedHueRange;
    Mat upperRedHueRange;
    inRange(hsv, cv::Scalar(0, 100, 100),
            cv::Scalar(10, 255, 255),
            lowerRedHueRange);
    inRange(hsv, cv::Scalar(160, 100, 100),
            cv::Scalar(179, 255, 255),
            upperRedHueRange);

    addWeighted(lowerRedHueRange, 1.0,
                upperRedHueRange, 1.0,
                0.0, binResultMask);
}

double PlaneControl::getEccentricity(const Moments &mu)
{
    double m20s02 = mu.m20 - mu.m02;
    double m20p02 = mu.m20 + mu.m02;

    double bigSqrt = sqrt(m20s02 * m20s02 + 4 * mu.m11 * mu.m11);
    return (m20p02 + bigSqrt) / (m20p02 - bigSqrt);
}

int PlaneControl::getCircleContours(Mat &frame, CircleContour &outContours)
{
    Mat redFiltered;
    getRedFilteredFrame(frame, redFiltered);

    CircleContour contours;
    findContours(redFiltered, contours,
                 RETR_TREE, CHAIN_APPROX_SIMPLE);

    if (contours.size() > 0)
    {
        for (int i = 0; i < contours.size(); i++)
        {
            // double contourEccentricity = getEccentricity(moments(contours[i]));
            // cout << contourEccentricity << endl;

            // if (contourEccentricity > MIN_CIRCLE_ECCENTRICITY)
            // {
                outContours.push_back(contours[i]);
            // }/

            if (m_isDebugRenderEnabled)
            {
                drawContours(frame, contours, i, Scalar(255, 0, 0), 3);
            }
        }
    }
}
