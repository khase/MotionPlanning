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

	int obs = obstacleInWay(obstacle, robot, nObst);

	if (obs > -1) {
		cout << "Aua";
		heading = heading.Neg();
		getroffen = true;
	}

	if (getroffen)
	{
		if (count > 30){
			return true;
		}
		count++;
	}


	
	actPoint.Mac(heading, DIST_MIN); // move next step
	robot[0].Set(actPoint);
	return false;
}

int Bug0::obstacleInWay(Box obstacle[], Box robot[], int nObst)
{
	class ShadowMatrix3x3 { public: double m[3][3]; };   // To allow inlining
	const Matrix3x3* Mat = (const Matrix3x3*)new ShadowMatrix3x3({0, -1, 0, 1, 0, 0, 0, 0, 1});

	Point out;
	for (int i = 0; i <nObst; i++)
	{
		double dist = obstacle[i].distance(robot[0], &out);
		if (dist <= DIST_MIN) {
			Point out2 = out * *Mat;
			out2.Normalize();
			headToPoint(out2);
			return i;
		}
	}

	return(-1);
}
