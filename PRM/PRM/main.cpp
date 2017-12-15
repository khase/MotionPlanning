#include "stdafx.h"
#include <iostream>
#include "cell.h"
#include <random>
#include <windows.h>

#include <vector>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>

using namespace std;
namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;
typedef std::pair<Eigen::VectorXd, unsigned> value;
typedef bg::model::point<Eigen::VectorXd, 1, bg::cs::cartesian> point;


/***********************************************************************************************************************************/
int _tmain(int argc, _TCHAR* argv[])
{
    WormCell cell;
    Eigen::VectorXd qStart(5), qGoal(5), q(5);
    vector<Eigen::VectorXd> path; // create a point vector for storing the path
    graph_t g;
    knn_rtree_t rtree;
    const float stepsize = .025f;

#define TEST_CASE 0
#ifdef TEST_CASE
#if TEST_CASE == 0
	// Example
	cout << "Example" << endl;
	qStart << 0., 0., 0., 0., 0.;
	qGoal << .6, .9, DEG2RAD(-90.), DEG2RAD(-180.), DEG2RAD(180.);

	Eigen::VectorXd segment(qGoal - qStart), delta(5);
	delta = segment.normalized() * stepsize;
	int steps = int(segment.norm() / stepsize);

	do
	{
		if (!cell.CheckPosition(qStart))
		{
			for (int i = 0; i < 10; ++i)
			{
				path.push_back(qStart);
				qStart += delta * .1f;
			}
		}
		else
		{
			path.push_back(qStart);
			qStart += delta;
		}
	} while (--steps > 0);

	path.push_back(qGoal);
	reverse(path.begin(), path.end());
	write_easyrob_program_file(path, "example.prg", false);
	path.clear();
	// !Example
#elif TEST_CASE == 1
	cout << "Test case 1" << endl;
	qStart << .6, .1, 0., 0., 0.;
    qGoal << .1, .8, DEG2RAD(-90.f), 0., 0.;
#elif TEST_CASE == 2
	cout << "Test case 2" << endl;
	qStart << .1, .8, DEG2RAD(-90.f), DEG2RAD(-180.f), DEG2RAD(180.f);
	qGoal << .9, .4, DEG2RAD(-90.f), DEG2RAD(-180.f), DEG2RAD(180.f);
#elif TEST_CASE == 3
	cout << "Test case 3" << endl;
	qStart << .9, .4, DEG2RAD(-90.f), DEG2RAD(-180.f), DEG2RAD(180.f);
	qGoal << .9, .75, DEG2RAD(-180.f), 0., 0.;
#elif TEST_CASE == 4
	cout << "Test case 4" << endl;
	qStart << .9, .75, DEG2RAD(-180.f), 0., 0.;
	qGoal << .5, .45, DEG2RAD(-180.f), 0., 0.;
#elif TEST_CASE == 5
	cout << "Test case 5" << endl;
	qStart << .5, .45, DEG2RAD(-180.f), 0., 0.;
	qGoal << .6, .95, DEG2RAD(-90.f), 0., 0.;
#elif TEST_CASE == 6
	cout << "Test case 6" << endl;
	qStart << .5, .45, DEG2RAD(-180.f), 0., 0.;
	qGoal << .6, .95, DEG2RAD(-90.f), 0., 0.;
#elif TEST_CASE == 7
	cout << "Test case 7 / colliding goal" << endl;
	qStart << .6, .95, DEG2RAD(-90.f), 0., 0.;
	qGoal << .7, .95, DEG2RAD(-90.f), 0., 0.;
#elif TEST_CASE == 8
	cout << "Test case 8 / colliding start" << endl;
	qStart << .7, .95, DEG2RAD(-90.f), 0., 0.;
	qGoal << .6, .95, DEG2RAD(-90.f), 0., 0.;
#elif TEST_CASE == 9
	cout << "Test case 9 / unreachable goal" << endl;
	qStart << .6, .95, DEG2RAD(-90.f), 0., 0.;
	qGoal << .6, 1.05, DEG2RAD(-90.f), 0., 0.;
#elif TEST_CASE == 10
	cout << "Test case 10 / unreachable start" << endl;
	qStart << .6, 1.05, DEG2RAD(-90.f), 0., 0.;
	qGoal << .6, .95, DEG2RAD(-90.f), 0., 0.;
#endif
#endif

	DWORD dwStart;
	DWORD dwElapsed;

	// Startzeit
	dwStart = GetTickCount();
    const int nNodes = 25000;
    // 1. step: building up a graph g consisting of nNodes vertices
    cout << "1. Step: building " << nNodes << " nodes for the graph" << endl;

	std::mt19937_64 generator = std::mt19937_64(std::random_device().operator()());
	std::uniform_real_distribution<double> random(0, 1);
	g = graph_t(nNodes);
	for (int index = 0; index < nNodes; index++) {
		// erstelle vertex Descriptor
		vertex_t vert = vertex_t(index);
		
		// generiere zufällige, kollisionsfreie Konfiguration
		g[vert].q_ = cell.NextRandomCfree();
		rtree_value val = rtree_value(cell.Robot(), vert);
		// Trage Konfiguration in kd-Tree ein
		rtree.insert(val);
	}

	// Zeit ausgeben ( in ms )
	dwElapsed = GetTickCount() - dwStart;
	cout << "took " << dwElapsed << " ms\n\n";

	// ###################	

	// Startzeit
	dwStart = GetTickCount();
    // 2. step: building edges for the graph, if the connection of 2 nodes are in free space
    cout << "2. Step: buildung edges for the graph" << endl;
	
	for (int index = 0; index < nNodes; index++) {
		// erstelle vertex Descriptor
		vertex_t vert = vertex_t(index);

		std::vector<rtree_value> nearest;
		MyWorm test = MyWorm(g[vert].q_);
		rtree.query(bgi::nearest(test, 5), std::back_inserter(nearest));

		/*for (auto &q : nearest)
		{
			cout << q.second << endl << endl;
		}
		cout << "-----" << endl;*/
	}

	// Zeit ausgeben ( in ms )
	dwElapsed = GetTickCount() - dwStart;
	cout << "took " << dwElapsed << " ms\n\n";

	// ###################

	// Startzeit
	dwStart = GetTickCount();
    // 3. Step: connecting start configuration to graph
    cout << "3. Step: connecting start configuration to graph" << endl;
	// Zeit ausgeben ( in ms )
	dwElapsed = GetTickCount() - dwStart;
	cout << "took " << dwElapsed << " ms\n\n";

	// ###################

	// Startzeit
	dwStart = GetTickCount();
    // 4. Step: connecting goal configuration to graph
    cout << "4. Step: connecting goal configuration to graph" << endl;
	// Zeit ausgeben ( in ms )
	dwElapsed = GetTickCount() - dwStart;
	cout << "took " << dwElapsed << " ms\n\n";

	// ###################

	// Startzeit
	dwStart = GetTickCount();
    // 5. Step: searching for shortest path
    cout << "5. Step: searching for shortest path" << endl;
	// Zeit ausgeben ( in ms )
	dwElapsed = GetTickCount() - dwStart;
	cout << "took " << dwElapsed << " ms\n\n";

    return EXIT_SUCCESS;
}
