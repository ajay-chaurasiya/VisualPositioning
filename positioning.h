//
// Created by ajay on 4/4/19.
//

#ifndef VISUALPOSITIONING_POSITIONING_H
#define VISUALPOSITIONING_POSITIONING_H

#include <opencv2/highgui.hpp>
#include <opencv/highgui.h>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <vector>

#include "aStarSearch.h"

using namespace std;
using namespace cv;

class positioning {
public:
    positioning(float PtoW[4][4], float CtoW[4][4], float PtoC [4][4], int grid[][COL]);
};


#endif //VISUALPOSITIONING_POSITIONING_H
