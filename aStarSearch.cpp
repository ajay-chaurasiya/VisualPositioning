//
// Created by ajay on 29/3/19.
//

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include "aStarSearch.h"
using namespace std;

// A Utility Function to check whether given cell (row, col)
// is a valid cell or not.
bool aStarSearch::isValid(int row, int col)
{
    // Returns true if row number and column number
    // is in range
    return (row >= 0) && (row < ROW) &&
           (col >= 0) && (col < COL);
}

// A Utility Function to check whether the given cell is
// blocked or not
bool aStarSearch::isUnBlocked(int grid[][COL], int row, int col)
{
    // Returns true if the cell is not blocked else false
    return (grid[row][col] == 1);
}

// A Utility Function to check whether destination cell has
// been reached or not
bool aStarSearch::isDestination(int row, int col, Pair dest)
{
    return (row == dest.first && col == dest.second);
}

// A Utility Function to calculate the 'h' heuristics.
double aStarSearch::calculateHValue(int row, int col, aStarSearch::Pair dest)
{
    // Return using the distance formula
    return ((double)sqrt ((row-dest.first)*(row-dest.first)
                          + (col-dest.second)*(col-dest.second)));
}

void aStarSearch::north(int &i, int &j, double &gNew, double &hNew, double &fNew,
                        Pair &dest, cell cellDetails[][COL], set<pPair> &openList, bool closedList[][COL], int grid[][COL], bool &foundDest, cv::Mat &map)
{
    int k, l;
    k = i - 1;
    l = j;

    directionProcess(i, j, k, l, gNew, hNew, fNew, dest, cellDetails, openList, closedList, grid, foundDest, map);
}

void aStarSearch::south(int &i, int &j, double &gNew, double &hNew, double &fNew,
                        Pair &dest, cell cellDetails[][COL], set<pPair> &openList, bool closedList[][COL], int grid[][COL], bool &foundDest, cv::Mat &map)
{
    int k, l;
    k = i + 1;
    l = j;

    directionProcess(i, j, k, l, gNew, hNew, fNew, dest, cellDetails, openList, closedList, grid, foundDest, map);
}

void aStarSearch::east(int &i, int &j, double &gNew, double &hNew, double &fNew,
                       Pair &dest, cell cellDetails[][COL], set<pPair> &openList, bool closedList[][COL], int grid[][COL], bool &foundDest, cv::Mat &map)
{
    int k, l;
    k = i;
    l = j + 1;

    directionProcess(i, j, k, l, gNew, hNew, fNew, dest, cellDetails, openList, closedList, grid, foundDest, map);
}

void aStarSearch::west(int &i, int &j, double &gNew, double &hNew, double &fNew,
                       Pair &dest, cell cellDetails[][COL], set<pPair> &openList, bool closedList[][COL], int grid[][COL], bool &foundDest, cv::Mat &map)
{
    int k, l;
    k = i;
    l = j - 1;

    directionProcess(i, j, k, l, gNew, hNew, fNew, dest, cellDetails, openList, closedList, grid, foundDest, map);
}

void aStarSearch::northEast(int &i, int &j, double &gNew, double &hNew, double &fNew,
                            Pair &dest, cell cellDetails[][COL], set<pPair> &openList, bool closedList[][COL], int grid[][COL], bool &foundDest, cv::Mat &map)
{
    int k, l;
    k = i - 1;
    l = j + 1;

    directionProcess(i, j, k, l, gNew, hNew, fNew, dest, cellDetails, openList, closedList, grid, foundDest, map);
}

void aStarSearch::northWest(int &i, int &j, double &gNew, double &hNew, double &fNew,
                            Pair &dest, cell cellDetails[][COL], set<pPair> &openList, bool closedList[][COL], int grid[][COL], bool &foundDest, cv::Mat &map)
{
    int k, l;
    k = i - 1;
    l = j - 1;

    directionProcess(i, j, k, l, gNew, hNew, fNew, dest, cellDetails, openList, closedList, grid, foundDest, map);
}

void aStarSearch::southEast(int &i, int &j, double &gNew, double &hNew, double &fNew,
                            Pair &dest, cell cellDetails[][COL], set<pPair> &openList, bool closedList[][COL], int grid[][COL], bool &foundDest, cv::Mat &map)
{
    int k, l;
    k = i + 1;
    l = j + 1;

    directionProcess(i, j, k, l, gNew, hNew, fNew, dest, cellDetails, openList, closedList, grid, foundDest, map);
}

void aStarSearch::southWest(int &i, int &j, double &gNew, double &hNew, double &fNew,
                            Pair &dest, cell cellDetails[][COL], set<pPair> &openList, bool closedList[][COL], int grid[][COL], bool &foundDest, cv::Mat &map)
{
    int k, l;
    k = i + 1;
    l = j - 1;

    directionProcess(i, j, k, l, gNew, hNew, fNew, dest, cellDetails, openList, closedList, grid, foundDest, map);
}

void aStarSearch::directionProcess(int &i, int &j, int &k, int &l, double &gNew, double &hNew, double &fNew,
                                   Pair &dest, cell cellDetails[][COL], set<pPair> &openList, bool closedList[][COL], int grid[][COL], bool &foundDest, cv::Mat &map)
{
    // Only process this cell if this is aStarSearch valid one
    if (isValid(k, l))
    {
        // If the destination cell is the same as the
        // current successor
        if (isDestination(k, l, dest))
        {
            // Set the Parent of the destination cell
            cellDetails[k][l].parent_i = i;
            cellDetails[k][l].parent_j = j;
            printf ("The destination cell is found\n");
            tracePath (cellDetails, dest, map);
            foundDest = true;
            return;
        }
            // If the successor is already on the closed
            // list or if it is blocked, then ignore it.
            // Else do the following
        else if (!closedList[k][l] &&
                 isUnBlocked(grid, k, l))
        {
            if ((k == i - 1 && l == j + 1) || (k == i - 1 && l == j - 1) || (k == i + 1 && l == j + 1) || (k == i + 1 && l == j - 1))
                gNew = cellDetails[i][j].g + 1.414;
            else
                gNew = cellDetails[i][j].g + 1.0;
            hNew = calculateHValue (k, l, dest);
            fNew = gNew + hNew;

            // If it isn’t on the open list, add it to
            // the open list. Make the current square
            // the parent of this square. Record the
            // f, g, and h costs of the square cell
            //			 OR
            // If it is on the open list already, check
            // to see if this path to that square is better,
            // using 'f' cost as the measure.
            if (cellDetails[k][l].f == FLT_MAX ||
                cellDetails[k][l].f > fNew)
            {
                openList.insert( make_pair(fNew,
                                           make_pair(k, l)));

                // Update the details of this cell
                cellDetails[k][l].f = fNew;
                cellDetails[k][l].g = gNew;
                cellDetails[k][l].h = hNew;
                cellDetails[k][l].parent_i = i;
                cellDetails[k][l].parent_j = j;
            }
        }
    }
}

// A Utility Function to trace the path from the source
// to destination
void aStarSearch::tracePath(cell cellDetails[][COL], Pair &dest, cv::Mat &map)
{
    printf ("\nThe Path is ");
    int row = dest.first;
    int col = dest.second;


    stack<Pair> Path;

    while (!(cellDetails[row][col].parent_i == row
             && cellDetails[row][col].parent_j == col ))
    {
        Path.push (make_pair (row, col));
        int temp_row = cellDetails[row][col].parent_i;
        int temp_col = cellDetails[row][col].parent_j;
        row = temp_row;
        col = temp_col;
    }

    Path.push (make_pair (row, col));
    while (!Path.empty())
    {
        pair<int,int> p = Path.top();
        Path.pop();
        printf("-> (%d,%d) ",p.first,p.second);
        map.at<uchar>(p.first,p.second) = 200;
    }

}

// A Function to find the shortest path between
// a given source cell to a destination cell according
// to A* Search Algorithm
    aStarSearch::aStarSearch(int grid[][COL], Pair &src, Pair &dest, cv::Mat &map)
{
    createMap(map,grid);
    // If the source is out of range
    if (!isValid (src.first, src.second))
    {
        printf ("Source is invalid\n");
        return;
    }

    // If the destination is out of range
    if (!isValid (dest.first, dest.second))
    {
        printf ("Destination is invalid\n");
        return;
    }

    // Either the source or the destination is blocked
    if (!isUnBlocked(grid, src.first, src.second) ||
        !isUnBlocked(grid, dest.first, dest.second))
    {
        printf ("Source or the destination is blocked\n");
        return;
    }

    // If the destination cell is the same as source cell
    if (isDestination(src.first, src.second, dest))
    {
        printf ("We are already at the destination\n");
        return;
    }

    // Create a closed list and initialise it to false which means
    // that no cell has been included yet
    // This closed list is implemented as a boolean 2D array
    bool closedList[ROW][COL];
    memset(closedList, false, sizeof (closedList));

    // Declare a 2D array of structure to hold the details
    //of that cell
    cell cellDetails[ROW][COL];

    int i, j;

    for (i=0; i<ROW; i++)
    {
        for (j=0; j<COL; j++)
        {
            cellDetails[i][j].f = FLT_MAX;
            cellDetails[i][j].g = FLT_MAX;
            cellDetails[i][j].h = FLT_MAX;
            cellDetails[i][j].parent_i = -1;
            cellDetails[i][j].parent_j = -1;
        }
    }

    // Initialising the parameters of the starting node
    i = src.first, j = src.second;
    cellDetails[i][j].f = 0.0;
    cellDetails[i][j].g = 0.0;
    cellDetails[i][j].h = 0.0;
    cellDetails[i][j].parent_i = i;
    cellDetails[i][j].parent_j = j;

    /*
    Create an open list having information as-
    <f, <i, j>>
    where f = g + h,
    and i, j are the row and column index of that cell
    Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1
    This open list is implenented as a set of pair of pair.*/
    set<pPair> openList;

    // Put the starting cell on the open list and set its
    // 'f' as 0
    openList.insert(make_pair (0.0, make_pair (i, j)));

    // We set this boolean value as false as initially
    // the destination is not reached.
    bool foundDest = false;

    while (!openList.empty())
    {
        pPair p = *openList.begin();

        // Remove this vertex from the open list
        openList.erase(openList.begin());

        // Add this vertex to the closed list
        i = p.second.first;
        j = p.second.second;
        closedList[i][j] = true;

        /*
            Generating all the 8 successor of this cell

                N.W N N.E
                \ | /
                \ | /
                W----Cell----E
                    / | \
                / | \
                S.W S S.E

            Cell-->Popped Cell (i, j)
            N --> North	 (i-1, j)
            S --> South	 (i+1, j)
            E --> East	 (i, j+1)
            W --> West		 (i, j-1)
            N.E--> North-East (i-1, j+1)
            N.W--> North-West (i-1, j-1)
            S.E--> South-East (i+1, j+1)
            S.W--> South-West (i+1, j-1)*/

        // To store the 'g', 'h' and 'f' of the 8 successors
        double gNew, hNew, fNew;

        if (!foundDest)
            north(i,j,gNew,hNew,fNew,dest,cellDetails,openList,closedList,grid,foundDest,map);
        if (!foundDest)
            south(i,j,gNew,hNew,fNew,dest,cellDetails,openList,closedList,grid,foundDest,map);
        if (!foundDest)
            east(i,j,gNew,hNew,fNew,dest,cellDetails,openList,closedList,grid,foundDest,map);
        if (!foundDest)
            west(i,j,gNew,hNew,fNew,dest,cellDetails,openList,closedList,grid,foundDest,map);
        if (!foundDest)
            northEast(i,j,gNew,hNew,fNew,dest,cellDetails,openList,closedList,grid,foundDest,map);
        if (!foundDest)
            northWest(i,j,gNew,hNew,fNew,dest,cellDetails,openList,closedList,grid,foundDest,map);
        if (!foundDest)
            southEast(i,j,gNew,hNew,fNew,dest,cellDetails,openList,closedList,grid,foundDest,map);
        if (!foundDest)
            southWest(i,j,gNew,hNew,fNew,dest,cellDetails,openList,closedList,grid,foundDest,map);
    }

    // When the destination cell is not found and the open
    // list is empty, then we conclude that we failed to
    // reach the destiantion cell. This may happen when the
    // there is no way to destination cell (due to blockages)
    if (!foundDest)
        printf("Failed to find the Destination Cell\n");
}

void aStarSearch::createMap(cv::Mat &map, int grid[][COL]) {
    for (int i = 0; i < ROW; i++)
    {
        for (int j = 0; j < COL; j++)
        {
            if (grid[i][j] == 0)
                map.at<uchar>(i,j) = 0;
        }
    }
//    map.at<uchar>(src.first,src.second) = 255;
}