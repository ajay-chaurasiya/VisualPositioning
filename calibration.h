//
// Created by ajay on 4/4/19.
//

#ifndef VISUALPOSITIONING_CALIBRATION_H
#define VISUALPOSITIONING_CALIBRATION_H

#include <opencv2/highgui.hpp>
#include <opencv/highgui.h>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <vector>

using namespace std;
using namespace cv;

class calibration {
public:
    calibration(float CtoW[4][4], float PtoC [4][4], float PtoW[4][4]);
};


#endif //VISUALPOSITIONING_CALIBRATION_H
