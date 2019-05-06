//
// Created by ajay on 29/3/19.
//

// A C++ Program to implement A* Search Algorithm
#include<bits/stdc++.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include "aStarSearch.h"

using namespace std;

// Driver program to test above function
int main()
{
    /* Description of the Grid-
    1--> The cell is not blocked
    0--> The cell is blocked */
    int grid[ROW][COL] =
            {
                    { 1, 0, 1, 1, 1, 1, 0, 1, 1, 1 },
                    { 1, 1, 1, 0, 1, 1, 1, 0, 1, 1 },
                    { 1, 1, 1, 0, 1, 1, 0, 1, 0, 1 },
                    { 0, 0, 1, 0, 1, 0, 0, 0, 0, 1 },
                    { 1, 1, 1, 0, 1, 1, 1, 0, 1, 0 },
                    { 1, 0, 1, 1, 1, 1, 0, 1, 0, 0 },
                    { 1, 0, 0, 0, 0, 1, 0, 0, 0, 1 },
                    { 1, 0, 1, 1, 1, 1, 0, 1, 1, 1 },
                    { 1, 1, 1, 0, 0, 0, 1, 0, 0, 1 }
            };

    // Mat object for representing map as an image
    cv::Mat map(9,10,CV_8UC1, cv::Scalar(100));

    // Source is the left-most bottom-most corner
    aStarSearch::Pair src = make_pair(8, 6);

    // Destination is the left-most top-most corner
    aStarSearch::Pair dest = make_pair(0, 0);

    // A* search algorithm to find the shortest path from source 'src' to destination 'dest'
    aStarSearch(grid, src, dest, map);

    return 0;
}