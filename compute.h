//
// Created by ajay on 4/4/19.
//

#ifndef VISUALPOSITIONING_COMPUTE_H
#define VISUALPOSITIONING_COMPUTE_H

#include <opencv2/highgui.hpp>
#include <opencv/highgui.h>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <vector>

using namespace std;
using namespace cv;

class compute {
public:

    static void detectAruco(Mat &image, Mat &imageCopy, vector<Vec3d> &rvecs, vector<Vec3d> &tvecs, Mat &cameraMatrix, Mat &distCoeffs, bool &tagFound);

    static void computePtoC(vector<Vec3d> &rvecs, vector<Vec3d> &tvecs, float PtoC[4][4], Mat &R);

    static void printPtoC(float PtoC[4][4]);

    static void computeInversePtoC(float PtoC [4][4], float invR[4][4], float invT[4][4], float inv[4][4], vector<Vec3d> &tvecs);

    static void computeCtoW(float CtoW[4][4], float PtoW[4][4], float inv[4][4]);

    static void printCtoW(float CtoW[4][4]);

    static void printRotationVector(vector<Vec3d> &rvecs);

    static void printTranslationVector(vector<Vec3d> &tvecs);

    static void compute3dPose(float PtoW[4][4], float CtoW[4][4], float PtoC [4][4]);

    static void printWorldCoordinates(float PtoW[4][4], Mat &imageCopy);

    static void printCameraCoordinates(float PtoC[4][4], Mat &imageCopy);

    static void initializeToZero(float RnT[4][4]);
};


#endif //VISUALPOSITIONING_COMPUTE_H
