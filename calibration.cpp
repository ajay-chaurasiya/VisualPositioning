//
// Created by ajay on 4/4/19.
//
#include <opencv2/highgui.hpp>
#include <opencv/highgui.h>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <vector>

#include "calibration.h"
#include "compute.h"

using namespace std;
using namespace cv;

calibration::calibration(float CtoW[4][4], float PtoC [4][4], float PtoW[4][4])
{
    float invR[4][4], invT[4][4], inv[4][4];

    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            invR[i][j] = 0;
            invT[i][j] = 0;
            inv[i][j] = 0;
            CtoW[i][j] = 0;
            if (i == j) invT[i][j] = 1;
        }
    }
    invR[3][3] = 1;

// Initialize Mat objects for storing image, camera matrix and distortion coffecients
Mat image, imageCopy;

Mat R = Mat::zeros(3,3,CV_64F);

vector<Vec3d> rvecs, tvecs;

vector<Vec3f> rotationAngles;
vector<Vec3f> translationMatrix;

// Initialize Mat objects for camera matrix and distortion coffecients
    Mat cameraMatrix, distCoeffs;

// Read the camera parameters from the specified file
    FileStorage fs("Camera_Param_720P.xml",FileStorage::READ);
    fs["Intrinsic_Parameters"] >> cameraMatrix;
    fs["Distorion_Coeffecients"] >> distCoeffs;

bool tagFound;

// Define the capture object for camera
 // start camera using the link
VideoCapture capture = VideoCapture("rtsp://ajay:ajay@192.168.1.108/cam/realmonitor?channel=1&subtype=0");

while (capture.grab())
    {
        capture >> image;  // Capture the frame from camera
        image.copyTo(imageCopy);
        compute::detectAruco(image, imageCopy, rvecs, tvecs, cameraMatrix, distCoeffs, tagFound);

        if (tagFound)
        {
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    invR[i][j] = 0;
                    invT[i][j] = 0;
                    inv[i][j] = 0;
                    CtoW[i][j] = 0;
                    if (i == j) invT[i][j] = 1;
                }
            }
            invR[3][3] = 1;

            compute::printRotationVector(rvecs);

            compute::printTranslationVector(tvecs);

            compute::computePtoC(rvecs, tvecs, PtoC, R);

            compute::printPtoC(PtoC);

            compute::computeInversePtoC(PtoC, invR, invT, inv, tvecs);

            compute::computeCtoW(CtoW, PtoW, inv);

            compute::printCtoW(CtoW);
        }

        // Display the image
        namedWindow("out", WINDOW_FREERATIO);
        imshow("out", imageCopy);

        char key = (char)waitKey(10);
        if (key == 27)
        break;
    }

// Release the camera
capture.release();
cvDestroyWindow("out");

}
