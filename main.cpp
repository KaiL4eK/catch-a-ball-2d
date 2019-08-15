#include <iostream>
#include <thread>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "simulator.hpp"
#include "control.hpp"

using namespace cv;
using namespace sim;
using namespace std;

const uint32_t MIX_APPROX_POINTS = 10;

int main(int argc, char **argv)
{
    CatchABallSimulator simulator(Size(800, 600), 0.001);

    PlaneControl control;

    vector<Point> circle_matches;

    int32_t prevShotIdx = simulator.getShotIdx();
    int32_t planeDistancePx = simulator.getPlaneDistancePx();

    while ( !simulator.is_end() )
    {
        simulator.tick();

        int32_t shotIdx = simulator.getShotIdx();
        /* Clear prediction if new shot happened */
        if ( shotIdx != prevShotIdx )
        {
            prevShotIdx = shotIdx;
            circle_matches.clear();
        }

        Mat control_frame = simulator.get_control_frame();
        if ( !control_frame.empty() )
        {
            int controlY = control.getPlanePositionPrediction(control_frame, planeDistancePx);

            simulator.setPlaneControl(controlY);
            
            imshow("control", control_frame);
        }

        imshow("full", simulator.getScene());
        waitKey(1);

        // this_thread::sleep_for(chrono::milliseconds(50));
    }
    
    return EXIT_SUCCESS;
}