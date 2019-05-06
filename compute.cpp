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

#include "compute.h"

using namespace std;
using namespace cv;

 void compute::detectAruco(Mat &image, Mat &imageCopy, vector<Vec3d> &rvecs, vector<Vec3d> &tvecs, Mat &cameraMatrix, Mat &distCoeffs, bool &tagFound)
 {
    // Define dictionary to be used for AruCo tags
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);

    // Initialize vectors to store corners and IDs of detected aruco tags
    vector<int> ids;
    vector<vector<Point2f>> corners;

    tagFound = false;
    aruco::detectMarkers(image, dictionary, corners, ids);  // Function to detect aruco markers

    if (ids.size() > 0)
    {
        aruco::drawDetectedMarkers(imageCopy, corners, ids);

        aruco::estimatePoseSingleMarkers(corners, 0.2, cameraMatrix, distCoeffs, rvecs, tvecs);

        for (int i = 0; i < ids.size(); i++)
            aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);

        if (ids.size() == 1)
            tagFound = true;
    }
}

void compute::computePtoC(vector<Vec3d> &rvecs, vector<Vec3d> &tvecs, float PtoC[4][4], Mat &R) {
    cout << "\n Rodrigues Matrix: \n";
    Mat rotVec = Mat::zeros(1, 3, CV_64F);
    rotVec.at<float>(0,1) = rvecs[0][1];
    rotVec.at<float>(0,2) = rvecs[0][2];
    rotVec.at<float>(0,3) = rvecs[0][3];
    Rodrigues(rvecs, R);
    for (int i = 0; i < 3; i++)
    {
        for (int j =0; j < 3; j++)
        {
            PtoC[i][j] = R.at<double>(i,j);
            cout << R.at<double>(i,j) << "\t \t";
            if (j ==2) cout << endl;
        }
    }
    cout << endl;
    PtoC[0][3] = tvecs[0][0];
    PtoC[1][3] = tvecs[0][1];
    PtoC[2][3] = tvecs[0][2];
    PtoC[3][3] = 1;
 }

 void compute::printPtoC(float PtoC[4][4]) {
     // Print Rotation Matrix
     cout << "Point to Camera RnT Matrix: \n";
     for (int i = 0; i < 4; i++)
     {
         for (int j =0; j<4; j++)
         {
             cout << PtoC[i][j] << "\t \t";
             if (j ==3) cout << endl;
         }
     }
 }

 void compute::computeInversePtoC(float PtoC [4][4], float invR[4][4], float invT[4][4], float inv[4][4], vector<Vec3d> &tvecs) {
     // Computing Inverse
     for (int i = 0; i < 4; i++)
     {
         for (int j = 0; j < 4; j++)
         {
             if (i > 2)
                 invR[i][j] = 0;
             else
                 invR[i][j] = PtoC[j][i];
             if (i == j && i == 3)
                 invR[i][j] = 1;
         }
     }

     invT[0][3] = -tvecs[0][0];
     invT[1][3] = -tvecs[0][1];
     invT[2][3] = -tvecs[0][2];

     for (int i = 0; i < 4; i++)
     {
         for (int j = 0; j < 4; j++)
         {
             for (int k = 0; k < 4; k++)
             {
                 inv[i][j] += invR[i][k] *  invT[k][j];
             }
         }
     }

     cout << endl << "inv(PtoC): " << endl;
     for (int i = 0; i < 4; i++)
     {
         for (int j = 0; j < 4; j++)
         {
             cout << inv[i][j] << " \t \t";
             if (j == 3) cout << endl;
         }
     }
 }

 void compute::computeCtoW(float CtoW[4][4], float PtoW[4][4], float inv[4][4]) {
     // Computing CtoW
     for (int i = 0; i < 4; i++)
     {
         for (int j = 0; j < 4; j++)
         {
             for (int k = 0; k < 4; k++)
             {
                 CtoW[i][j] += PtoW[i][k] *  inv[k][j];
             }
         }
     }
 }

 void compute::printCtoW(float CtoW[4][4]) {
     // Print CtoW matrix
     cout << "CtoW: \n";
     for (int i = 0; i < 4; i++)
     {
         for (int j = 0; j < 4; j++)
         {
             cout << CtoW[i][j] << " \t ";
             if (j == 3) cout << endl;
         }
     }
 }

void compute::printRotationVector(vector<Vec3d> &rvecs) {
    cout << endl << "Rotation Vetor: " << endl;
    for (const auto i : rvecs)
    {
        cout << i << ' ' ;
    }
}

void compute::printTranslationVector(vector<Vec3d> &tvecs) {
    cout << endl << "Translation Vector: " << endl;
    for (const auto i : tvecs)
        cout << i << ' ' ;
}

void compute::compute3dPose(float PtoW[4][4], float CtoW[4][4], float PtoC [4][4]) {
    // Compute 3D Pose of the Aruco Tag
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                PtoW[i][j] += CtoW[i][k] * PtoC[k][j];
            }
        }
    }
 }

void compute::printWorldRotationVector(Mat &R, float PtoW[4][4]) {
    Mat G = R.clone();
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            G.at<double>(i,j) = PtoW[i][j];
        }
    }
    float sy = sqrt(G.at<double>(0,0) * G.at<double>(0,0) +  G.at<double>(1,0) * G.at<double>(1,0) );

    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular)
    {
        x = atan2(G.at<double>(2,1) , G.at<double>(2,2));
        y = atan2(-G.at<double>(2,0), sy);
        z = atan2(G.at<double>(1,0), G.at<double>(0,0));
    }
    else
    {
        x = atan2(-G.at<double>(1,2), G.at<double>(1,1));
        y = atan2(-G.at<double>(2,0), sy);
        z = 0;
    }

    cout << "World Rotation Vector: \t" << (x*180/CV_PI) << "\t"  << (y*180/CV_PI) << "\t" << (z*180/CV_PI) << endl;
}

 void compute::printWorldCoordinates(float PtoW[4][4], Mat &imageCopy) {
     // Store the coordinates of Pw in string form to print on the image
     char xw[10], yw[10], zw[10];

     sprintf(xw, "X: %f m", PtoW[0][3]);
     sprintf(yw, "Y: %f m", PtoW[1][3]);
     sprintf(zw, "Z: %f m", PtoW[2][3]);

     // Print the Pw coordinates on the image
     putText(imageCopy, "World Coordinates of Robot", Point(7,20), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(255,255,0));
     putText(imageCopy, xw, Point(7,40), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(255,255,255));
     putText(imageCopy, yw, Point(7,60), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(255,255,255));
     putText(imageCopy, zw, Point(7,80), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(255,255,255));
 }

 void compute::printCameraCoordinates(float PtoC[4][4], Mat &imageCopy) {
     // Store the coordinates of Pc in string form to print on the image
     char xc[10], yc[10], zc[10];

     sprintf(xc, "X: %f m", PtoC[0][3]);
     sprintf(yc, "Y: %f m", PtoC[1][3]);
     sprintf(zc, "Z: %f m", PtoC[2][3]);

     // Print the Pc coordinates on the image
     putText(imageCopy, "Camera Coordinates of Robot", Point(7,100), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0,255,255));
     putText(imageCopy, xc, Point(7,120), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(255,255,255));
     putText(imageCopy, yc, Point(7,140), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(255,255,255));
     putText(imageCopy, zc, Point(7,160), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(255,255,255));
 }

 void compute::initializeToZero(float RnT[4][4]) {
     for (int i = 0; i < 4; i++)
         for (int j = 0; j < 4; j++)
             RnT[i][j] = 0;
 }