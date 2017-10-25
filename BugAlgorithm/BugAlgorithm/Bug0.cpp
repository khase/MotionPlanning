#include "Bug0.h"

Bug0::Bug0(const string& name)
	: BugAlgorithm(name)
{


}


Bug0::~Bug0()
{
}

void Bug0::headToPoint(Point dest)
{
	this->heading = (dest - this->actPoint).Normalize();
}

void Bug0::headToGoal()
{
	headToPoint(this->goalPosition);
}

bool Bug0::update(Box obstacle[], Box robot[], int nObst)
{
	Point pt, robotPos = actPoint;
	static Box mink_diff;
	static int ret = -1;  // no obstacle

	if (goalReached(robotPos, goalPosition, DIST_MIN)) {
		actPoint = goalPosition;
		cout << "at goal, smile :) \n";
		return true;
	}

	// test if we are near (dist <= MIN_DIST) an obstacle
	// this function also sets a new heading along the obstacles wall if it returns true
	int obs = obstacleInWay(obstacle, robot, nObst);

	if (obs > -1) {
		cout << "follow obstacle" << endl;
		// activate emergency exit
		getroffen = true;
	}
	else {
		cout << "heading to goal" << endl;
		// reset heading towards goal
		headToGoal();
	}

	if (getroffen)
	{
		if (count > 10000){
			// we took a huge ammount of iterations
			// assume the algorithm broken and exit
			cout << "Fuck man, wo ist der Asgang..." << endl;
			return true;
		}
		count++;
	}


	
	actPoint.Mac(heading, DIST_MIN); // move next step
	// set the robots actual position
	robot[0].Set(actPoint);
	return false;
}

int Bug0::obstacleInWay(Box obstacle[], Box robot[], int nObst)
{
	class ShadowMatrix3x3 { public: double m[3][3]; };   // To allow inlining
	const Matrix3x3* Mat = (const Matrix3x3*)new ShadowMatrix3x3({0, -1, 0, 1, 0, 0, 0, 0, 1}); // simple 90° rotation Matrix

	// this will point to the closest hitpoint of the obstacle
	// should always be rectangular to the targets wall
	Point out;
	for (int i = 0; i <nObst; i++)
	{
		double dist = obstacle[i].distance(robot[0], &out);
		// Run along the obstacles wall till the distance rises
		if (dist <= DIST_MIN) {
			// rotate hitpoint vector by 90 degree
			Point out2 = out * *Mat;
			out2.Normalize();
			// should always be perpendicular to the obstavles wall
			this->heading = out2;
			return i;
		}
	}

	return(-1);
}
