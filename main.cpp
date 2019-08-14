#include <iostream>
#include <thread>

#include <opencv2/highgui/highgui.hpp>

#include "simulator.hpp"

using namespace cv;
using namespace sim;
using namespace std;

int main(int argc, char **argv)
{
    CatchABallSimulator simulator(Size(800, 600));

    while ( !simulator.is_end() )
    {
        simulator.tick();
        simulator.show_scene();

        Mat control_frame = simulator.get_control_frame();
        imshow("control", control_frame);
        waitKey(1);

        // this_thread::sleep_for(chrono::milliseconds(10));
    }
    

    return EXIT_SUCCESS;
}