#include <iostream>
#include <thread>
#include <vector>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "simulator.hpp"
#include "control.hpp"

using namespace cv;
using namespace sim;
using namespace std;

void renderStats(Mat &frame, CatchABallStatistics &stats)
{
    double successRate = static_cast<double>(stats.ballsCatched) / 
                            (stats.shotsDone - stats.flewAway - 1);

    vector<string> stringsToRender;
    stringsToRender.push_back("Ball catches: " + to_string(stats.ballsCatched));
    stringsToRender.push_back("Shots done: " + to_string(stats.shotsDone));
    stringsToRender.push_back("Flew away: " + to_string(stats.flewAway));
    stringsToRender.push_back("Success rate: " + to_string(successRate));

    for (size_t i = 0; i < stringsToRender.size(); i++)
    {
        putText(frame, stringsToRender[i], Point2f(10, (i+1)*15), 
                FONT_HERSHEY_PLAIN, 1.3,  Scalar(0,0,255));
    }
}

const string argsKeys =
    "{help h usage ? |              | Print this message  }"
    "{config         | config.yaml  | Path to config file }"
    ;

int main(int argc, char **argv)
{
    CommandLineParser parser(argc, argv, argsKeys);
    
    if (parser.has("help"))
    {
        parser.printMessage();
        return EXIT_SUCCESS;
    }

    if (!parser.check())
    {
        parser.printErrors();
        return EXIT_SUCCESS;
    }

    string configFpath = parser.get<string>("config");

    CatchABallSimulator simulator(configFpath);
    simulator.setAutoShootingMode(true);

    PlaneControl control;

    int32_t shotsCntr = 0;
    int32_t planeDistancePx = simulator.getPlaneDistancePx();

    while ( !simulator.isEnd() )
    {
        simulator.tick();
        CatchABallStatistics stats = simulator.getStatistics();

        /* Clear prediction if new shot happened */
        if ( stats.shotsDone != shotsCntr )
        {
            shotsCntr = stats.shotsDone;
            control.resetPredictions();
        }

        Mat control_frame = simulator.getControlFrame();
        if ( !control_frame.empty() )
        {
            double ballRadiusPx = control.getAveragePredictedRadiusPx();

            int controlY = control.getPlanePositionPrediction(control_frame, 
                                        planeDistancePx-ballRadiusPx);          

            simulator.setPlaneControl(controlY);
            
            imshow("control", control_frame);
        }

        Mat mainScene = simulator.getScene();
        renderStats(mainScene, stats);
        imshow("experiment", mainScene);
        waitKey(1);

        // this_thread::sleep_for(chrono::milliseconds(10));
    }
    
    return EXIT_SUCCESS;
}