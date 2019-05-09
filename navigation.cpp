//
// Created by ajay on 7/5/19.
//

// This program will run on the robot

#include <iostream>
#include <cstdlib>

using namespace std;

int main()
{
//   Variables to store destination x and y grid positions
    int dx, dy;
//   Variables for storing old (current) position and new position
    int xo, yo, xn, yn;
//    Varables to store old (current) and new angles
    int an, ao;
//    Variables to store heading direction angles
    int px = 0,
            nx = 180,
            py = 90,
            ny = 270,
            pxpy = 45,
            nxpy = 135,
            nxny = 225,
            pxny = 315;

    // an = 0, ao = 90;
    // xn = 5, xo = yn = yo = dy = 0;
    // dx = 5;

    cout << "\n Enter source: \t";
    cin >> xo >> yo;
    cout << "\n Enter destination: \t";
    cin >> xn >> yn;
    cout << "\n Enter current angle of robot: \t";
    cin >> ao;
    dx = xn;
    dy = yn;

    //        Conditional statements to determine the new direction of robot
    if (xn > xo && yn == yo) {
        an = px;
    }

    else if (xn < xo && yn == yo) {
        an = nx;
    }

    else if (xn == xo && yn > yo) {
        an = py;
    }

    else if (xn == xo && yn < yo) {
        an = ny;
    }

    else if (xn > xo && yn > yo) {
        an = pxpy;
    }

    else if (xn < xo && yn > yo) {
        an = nxpy;
    }

    else if (xn > xo && yn < yo) {
        an = pxny;
    }

    else if (xn < xo && yn < yo) {
        an = nxny;
    }


//    conditional loop to check the direction of robot of target and rectify it
    if (abs(an - ao) >= 5) {
		if (ao > an) {
			if ((ao - an) <= 180) {
				cout << "Rotate CW \n";
			}
			else if ((ao -an) > 180) {
				cout << "Rotate CCW \n";
			}
		}
		if (an > ao) {
			if ((an - ao) <= 180) {
				cout << "Rotate CCW \n";
			}
			else if ((an - ao) > 180) {
				cout << "Rotate CW \n";
			}
		}
	}

//    Conditional loop to check if robot has reached destination else move the robot to reach
    if (xo != dx || yo != dy) {
        cout << "Moving towards destination." << endl;

//        switch statements to execute forward movement of robot in respective destination direction
        switch (an)
        {
            case 0:    cout << "Positive X direction" << endl;
                break;

            case 180:    cout << "Negative X direction" << endl;
                break;

            case 90:    cout << "Positive Y direction" << endl;
                break;

            case 270:    cout << "Negative Y direction" << endl;
                break;

            case 45:  cout << "Positive X Positive Y direction" << endl;
                break;

            case 135:  cout << "Negative x Positive Y direction" << endl;
                break;

            case 225:  cout << "Negative X Negative Y direction" << endl;
                break;

            case 315:  cout << "Positive X Negative Y direction" << endl;
                break;

            case 360:   cout << "Program Terminated!" << endl;
                break;

            default:    cout << "Default" << endl;
                break;
        }
    }
}