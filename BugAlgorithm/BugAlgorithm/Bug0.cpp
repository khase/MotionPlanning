#include "Bug0.h"



Bug0::Bug0(const string& name)
	: BugAlgorithm(name)
{
}


Bug0::~Bug0()
{
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
	}
	
	actPoint.Mac(heading, DIST_MIN); // move next step
	robot[0].Set(actPoint);
	return false;
}

int Bug0::obstacleInWay(Box obstacle[], Box robot[], int nObst)
{
	Point out;
	for (int i = 0; i <nObst; i++)
	{
		double dist = obstacle[i].distance(robot[0], &out);
		if (dist <= DIST_MIN) {
			return i;
		}
	}

	return(-1);
}