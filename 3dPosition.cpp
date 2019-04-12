//
// Created by ajay on 4/4/19.
//

//#include <opencv/cv.hpp>
//#include <opencv2/core.hpp>
//#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv/highgui.h>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
//#include <opencv2/videoio.hpp>

//#include <sstream>
#include <iostream>
//#include <fstream>
#include <vector>
//#include <stdio.h>

#include "calibration.h"
#include "positioning.h"
//#include "compute.h"

using namespace std;
using namespace cv;

int main()
{
    float CtoW[4][4], PtoC[4][4];
    float PtoW[4][4] = { 1,  0,  0,     0,
                         0,  1,  0,     0,
                         0,  0,  1,      0,
                         0,  0,  0,      1};
    calibration(CtoW, PtoC, PtoW);

    positioning(PtoW, CtoW, PtoC);

    return 0;
}