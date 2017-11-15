#include <iostream>
#include <math.h>
#include "Potential.h"

using namespace std;

/*********************************************************************************************************************************/
static const double INKR = 0.01;        // step size for robot along gradient
static const double DIST_MIN = 0.05;    // minimum distance between the robot and the goal
static const double GOAL_ERROR = 0.01;  // distance between the robot and the goal
static const double DIST_MIN_OBST = 0.1;   // distance when the obstacle interferes with the robot
static const double DIST_MIN_GOAL = 0.5;    // distance something
static const double OBST_FORCE_SCALE = 0.00001;    // Magic do not touch
static const double GOAL_FORCE_SCALE = 1;    // Magic do not touch
static const double NAV_K = 20;



/*********************************************************************************************************************************/
Potential::Potential(const std::string& name)
    : goalPosition(1.0, 1.0, 0.0) //set the goal position
    , startPosition(0.0, 0.0, 0.0) //set the start position
{
}

void Potential::setGoalPosition(double x, double y, double z)
{
    goalPosition.x = x;
    goalPosition.y = y;
    goalPosition.z = z;
}

void Potential::setStartPosition(double x, double y, double z)
{
    startPosition.x = x;
    startPosition.y = y;
    startPosition.z = z;
}

void Potential::setActPoint(Point p)
{
    actPoint.x = p.x;
    actPoint.y = p.y;
    actPoint.z = p.z;
}

Point Potential::getGoalPosition()
{
    return goalPosition;
}

Point Potential::getStartPosition()
{
    return startPosition;
}

Point Potential::getRobPos()
{
    return actPoint;
}

bool Potential::goalReached(Point robotPos, Point goalPos, double distError)
{
    return (robotPos.Distance(goalPos) <= distError);
}

/*************************************************************************************************************************/
bool Potential::update_box(Box obstacle[], Box robot[], int nObst)
{
    Point robotPos = actPoint;
    static int cnt = 0;

    cnt++;

    if (goalReached(robotPos, goalPosition, GOAL_ERROR))
    {
        actPoint = goalPosition;
        std::cout << "at goal, smile :)\n";

        return true;
    }

	Point globalForceVector = (goalPosition - robotPos);
	double dGoal = robotPos.Distance(goalPosition);

	if (dGoal > DIST_MIN_GOAL) {
		globalForceVector = (DIST_MIN_GOAL * GOAL_FORCE_SCALE * globalForceVector) / dGoal;
	}

	Point repulsive = Point(0, 0, 0);
	for (int i = 0; i < nObst; i++) {
		Box obst = obstacle[i];
		Point localForceVector;
		double dist = robot[0].distance(obst, &localForceVector);
		if (dist <= DIST_MIN_OBST) {
			localForceVector = localForceVector.Normalize();
			Point localRepulsive = OBST_FORCE_SCALE * ((1 / DIST_MIN_OBST) - (1 / dist)) * (1 / (dist * dist)) * localForceVector;
			repulsive += localRepulsive;
		}
	}

	globalForceVector += repulsive;

    actPoint.Mac(globalForceVector.Normalize(), INKR); // move next step
	robot[0].Set(actPoint);
    return false;
}

/*************************************************************************************************************************/
bool Potential::update_cylinder(Cylinder obstacle[], Cylinder robot[], int nObst)
{

	Point robotPos = actPoint;
	static int cnt = 0;

	cnt++;

	if (goalReached(robotPos, goalPosition, GOAL_ERROR))
	{
		actPoint = goalPosition;
		std::cout << "at goal, smile :)\n";

		return true;
	}

	Point globalForceVector = (goalPosition - robotPos);
	double dGoal = robotPos.Distance(goalPosition);

	if (dGoal > DIST_MIN_GOAL) {
		globalForceVector = (DIST_MIN_GOAL * GOAL_FORCE_SCALE * globalForceVector) / dGoal;
	}

	Point repulsive = Point(0, 0, 0);
	for (int i = 0; i < nObst; i++) {
		Cylinder obst = obstacle[i];
		Point localForceVector;
		double dist = robot[0].distance(obst, &localForceVector);
		if (dist <= DIST_MIN_OBST) {
			localForceVector = localForceVector.Normalize();
			Point localRepulsive = OBST_FORCE_SCALE * ((1 / DIST_MIN_OBST) - (1 / dist)) * (1 / (dist * dist)) * localForceVector;
			repulsive += localRepulsive;
		}
	}

	globalForceVector += repulsive;

	actPoint.Mac(globalForceVector.Normalize(), INKR); // move next step
	robot[0].SetCenter(actPoint);
	return false;
}

/*************************************************************************************************************************/

double calc_beta_i(Cylinder obstacle, Cylinder robot, int i) {
	Point pt = robot.GetCenter();
	if (i == 0) {
		return -pt.Sub(obstacle.GetCenter()).SquareMagnitude()
			+ pow(obstacle.GetRadius() + robot.GetRadius(), 2);
	}
	else {
		return pt.Sub(obstacle.GetCenter()).SquareMagnitude()
			- pow(obstacle.GetRadius() + robot.GetRadius(), 2);
	}
}

double calc_beta(Cylinder obstacle[], Cylinder robot, int nObst) {
	double res = 1;
	for (int i = 0; i < nObst; i++) {
		res *= calc_beta_i(obstacle[i], robot, i);
	}
	return res;
}

Point calc_p_beta_i(Cylinder obstacle, Cylinder robot, int i) {
	Point pt = obstacle.GetCenter() - robot.GetCenter();
	pt *= 2;
	if (i == 0) {
		pt *= -1;
	}
	return pt;
}

Point calc_p_beta(Cylinder obstacle[], Cylinder robot, int nObst) {
	Point res = Point(0, 0, 0);
	for (int i = 0; i < nObst; i++) {
		Point pt = calc_p_beta_i(obstacle[i], robot, i);
		double tmp = 1;
		for (int j = 0; j < nObst; j++) {
			if (j == i) {
				continue;
			}
			tmp *= calc_beta_i(obstacle[j], robot, j);
		}
		pt *= tmp;
		res += pt;
	}
	return res;
}

bool Potential::update_cylinder_navigation(Cylinder obstacle[], Cylinder robot[], int nObst)
{
    Point robotPos = actPoint;
    static int cnt = 0;

    cnt++;

    if (goalReached(robotPos, goalPosition, GOAL_ERROR))
    {
        actPoint = goalPosition;
        std::cout << "at goal, smile :)\n";

        return true;
    }

	double k = NAV_K;
	Point qminusgoal = goalPosition - robot[0].GetCenter();
	double distgoal = qminusgoal.SquareMagnitude();
	double beta = calc_beta(obstacle, robot[0], nObst);
	Point p_beta = calc_p_beta(obstacle, robot[0], nObst);

	double distGoalBeta = abs(pow(distgoal, 2 * k) + beta);


	Point heading =
		(
			2 * qminusgoal
			* pow(distGoalBeta, 1 / k)
			- pow(distgoal, 2)  * (1 / k) * pow(distGoalBeta, (1 / k) - 1)
			* (2 * k * pow(distgoal, (2 * k) - 2) * qminusgoal + p_beta)
		) / (
			pow(distGoalBeta, 2 / k)
		);

    actPoint.Mac(heading.Normalize(), INKR); // move next step
	robot[0].SetCenter(actPoint);
    return false;
}
