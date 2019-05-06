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
#include <time.h>

#include "positioning.h"
#include "compute.h"

using namespace std;
using namespace cv;

positioning::positioning(float PtoW[4][4], float CtoW[4][4], float PtoC [4][4])
{
    // Define the 'capture' object for camera
    // start camera using the link specified
    VideoCapture capture = VideoCapture("rtsp://ajay:ajay@192.168.1.108/cam/realmonitor?channel=1&subtype=0");
    int frameWidth = capture.get(CV_CAP_PROP_FRAME_WIDTH);
    int frameHeight = capture.get(CV_CAP_PROP_FRAME_HEIGHT);
    int frameCount = 15;    //capture.get(CV_CAP_PROP_FPS);

    cout << "Width: " << frameWidth << "Height: " << frameHeight << "FPS: " << frameCount;
//    VideoWriter video("PoseEstimation_Trial_720P_FPS_15_Trial_5.avi", CV_FOURCC('M','J','P','G'), frameCount, Size(frameWidth, frameHeight));

    compute::initializeToZero(PtoW);

    // Initialize Mat objects for storing image, camera matrix and distortion coffecients
    Mat image, imageCopy;
    Mat R = Mat::zeros(3, 3, CV_64F);

    // Initialize Mat objects for camera matrix and distortion coffecients
    Mat cameraMatrix, distCoeffs;

    vector<Vec3d> rvecs, tvecs;

    // Read the camera parameters from the specified file
    FileStorage fs("Camera_Param_720P.xml",FileStorage::READ);
    fs["Intrinsic_Parameters"] >> cameraMatrix;
    fs["Distorion_Coeffecients"] >> distCoeffs;

    bool tagFound;

    clock_t time;

    while (capture.grab())
     {
         time = clock();
        capture >> image;  // Capture the frame from camera
        image.copyTo(imageCopy);
        compute::detectAruco(image, imageCopy, rvecs, tvecs, cameraMatrix, distCoeffs, tagFound);

        if (tagFound)
        {
            compute::computePtoC(rvecs, tvecs, PtoC, R);

            compute::compute3dPose(PtoW, CtoW, PtoC);

            compute::printWorldRotationVector(R, PtoW);

//            compute::initializeToZero(PtoC);
        }

         compute::printWorldCoordinates(PtoW, imageCopy);

         compute::printCameraCoordinates(PtoC, imageCopy);

         // Display the image
         namedWindow("3D Pose Estimation Window", WINDOW_FREERATIO);
         imshow("3D Pose Estimation Window", imageCopy);

         // Store Video Stream
//         video.write(imageCopy);

        compute::initializeToZero(PtoW);

        time = clock() - time;
        double functionTime = ((double)time)/CLOCKS_PER_SEC;
        cout << "Positioning time taken = " << functionTime << endl;

         char key = (char) waitKey(5);
         if (key == 27)
         {
             // Release the camera
             capture.release();
//             video.release();
             destroyAllWindows();
             break;
         }
     }
}
