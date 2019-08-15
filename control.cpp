#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "control.hpp"

using namespace cv;
using namespace std;

#include <Eigen/QR>

static void polyfit(const vector<double> &xv, 
                    const vector<double> &yv, 
                    vector<double> &coeff, 
                    int order)
{
	Eigen::MatrixXd A(xv.size(), order+1);
	Eigen::VectorXd yv_mapped = Eigen::VectorXd::Map(&yv.front(), yv.size());
	Eigen::VectorXd result;

	assert(xv.size() == yv.size());
	assert(xv.size() >= order+1);

	for (size_t i = 0; i < xv.size(); i++)
	for (size_t j = 0; j < order+1; j++)
		A(i, j) = pow(xv.at(i), j);

	result = A.householderQr().solve(yv_mapped);

	coeff.resize(order+1);
	for (size_t i = 0; i < order+1; i++)
		coeff[i] = result[i];
}

int PlaneControl::getPlanePositionPrediction(Mat &ballFrame, int plabeDistPx)
{
    CircleMatch match;

    Mat red_filtered = getRedCircleFilteredFrame(ballFrame);

    if ( getCircle(red_filtered, match) == 0 )
    {
        circle_matches.push_back(match.pos);
    }

    /* Approximate previous points to predict future trajectory */
    if ( circle_matches.size() > MIX_APPROX_POINTS )
    {
        vector<double> vx(circle_matches.size());
        vector<double> vy(circle_matches.size());

        for ( int i = 0; i < circle_matches.size(); i++ )
        {
            vx[i] = circle_matches[i].x;
            vy[i] = circle_matches[i].y;
        }

        vector<double> coeffs;
        polyfit(vx, vy, coeffs, 2);

        assert(coeffs.size() == 3);

        for (int x = 0; x < ballFrame.size().width; x++)
        {
            int y = coeffs[0] + x * coeffs[1] + x*x *coeffs[2];

            circle(ballFrame,
                    Point(x, y),
                    1,
                    Scalar(0, 0, 128),
                    FILLED);
        }

        int controlY = coeffs[0] + 
                        planeDistancePx * coeffs[1] + 
                        planeDistancePx*planeDistancePx * coeffs[2];
        
        simulator.setPlaneControl(controlY);
    }

    for ( auto pnt_mtc : circle_matches )
    {
        circle(ballFrame,
                pnt_mtc,
                2,
                Scalar(255, 0, 128),
                FILLED);
    }
}

Mat PlaneControl::getRedCircleFilteredFrame(Mat &frame)
{
    /* In real situation GaussianBlur() required */ 
    Mat hsv;
    cvtColor(frame, hsv, COLOR_BGR2HSV);

    Mat lower_red_hue_range;
    Mat upper_red_hue_range;
    inRange(hsv, cv::Scalar(0, 100, 100), 
                 cv::Scalar(10, 255, 255), lower_red_hue_range);
    inRange(hsv, cv::Scalar(160, 100, 100), 
                 cv::Scalar(179, 255, 255), upper_red_hue_range);

    Mat bin_mask_4_red;
    addWeighted(lower_red_hue_range, 1.0, 
                upper_red_hue_range, 1.0, 
                0.0, bin_mask_4_red);
    
    return bin_mask_4_red;
}

int PlaneControl::getCircles(Mat &bin_balls, vector<CircleMatch> &circles)
{
    vector<vector<Point>> contours;
    findContours(bin_balls, contours, 
                    RETR_TREE, CHAIN_APPROX_SIMPLE);
    
    if (contours.size() > 0)
    {
        int maxContourIdx = -1;
        double maxContourArea = -1;

        for( int i = 0; i< contours.size(); i++ )
        {
            // drawContours( control_frame, 
            // contours, 
            // i, 
            // Scalar( 255, 0, 0 ) );

            CircleMatch match;

            minEnclosingCircle(contours[maxContourIdx], 
                                match.pos, 
                                match.radius);

            if ( match.radius < 10 )
                continue;

            circles.push_back(match);
        }

        return 0;
    }

    return -1;
}
