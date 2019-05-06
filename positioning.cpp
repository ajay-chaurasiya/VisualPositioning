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
#include "aStarSearch.h"

using namespace std;
using namespace cv;

positioning::positioning(float PtoW[4][4], float CtoW[4][4], float PtoC [4][4], int grid[][COL])
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

//    variable to store frame count to enter path planning mode only once in a while
    int frame=0;
//    Mat object for representing map as an image
    cv::Mat map(9,10,CV_8UC1, cv::Scalar(100));

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

         if(frame == 4)
             map=(9,10,CV_8UC1, cv::Scalar(100));
         if(frame == 5)
         {
//             Initialize coordinate values for source
             int x = (PtoW[0][3])/0.6, y = (PtoW[1][3])/0.6;
//             Source
             aStarSearch::Pair src = make_pair(x, y);
//             Destination
             aStarSearch::Pair dest = make_pair(0,0);
//             A* search algorithm to find the shortest path from source 'src' to destination 'dest'
             aStarSearch(grid, src, dest, map);
             cv::namedWindow("Map", cv::WINDOW_FREERATIO);
             cv::imshow("Map",map);
//             cv::waitKey(0);
//             cv::destroyAllWindows();
             frame = 0;
         }

         // Store Video Stream
//         video.write(imageCopy);

        compute::initializeToZero(PtoW);

         frame++;

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
