#include "stdafx.h"
#include <iostream>
#include "cell.h"
#include <random>
#include <windows.h>
#include <thread>

#include <vector>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/config.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <utility>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>

using namespace std;
namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;
typedef std::pair<Eigen::VectorXd, unsigned> value;
typedef bg::model::point<Eigen::VectorXd, 1, bg::cs::cartesian> point;

int addEdges(int num, int start, int end, graph_t &g, knn_rtree_t &rtree)
{
	WormCell cell;
	int edges = 0;
	for (int index = start; index < end; index++) {
		if (index % 500 == 0) {
			std::cout << ".";
		}
		vertex_t vert = vertex_t(index);
		Eigen::VectorXd actVector = g[vert].q_;
		std::vector<rtree_value> nearest;
		MyWorm test = MyWorm(actVector);
		rtree.query(bgi::nearest(test, 10), std::back_inserter(nearest));


		for (auto &q : nearest)
		{
			Eigen::VectorXd nearestVector = g[q.second].q_;
			if (!boost::edge(q.second, vert, g).second && cell.CheckMotion(nearestVector, actVector)) {
				float lengthOfEdge = (nearestVector - actVector).norm();
				boost::add_edge(q.second, vert, lengthOfEdge, g);
				edges++;
			}
		}
	}
	return 0;
}

void write_gnuplot_file2(graph_t g, string filename)
{
	ofstream myfile;
	myfile.open(filename);

	// Iterate through the edges and print them out
	typedef boost::graph_traits<graph_t>::edge_iterator edge_iter;
	edge_iter ei, ei_end;

	int cnt = 0; // edge counter

	for (tie(ei, ei_end) = edges(g); ei != ei_end; ++ei)
	{
		myfile << g[ei->m_source].q_[0] << " " << g[ei->m_source].q_[1] << endl;
		myfile << g[ei->m_target].q_[0] << " " << g[ei->m_target].q_[1] << endl << endl;
		cnt++;
	}

	cout << "Number of edges in " << filename << ": " << cnt << endl;
	myfile.close();
}

/***********************************************************************************************************************************/
int _tmain(int argc, _TCHAR* argv[])
{
	DWORD dwStartTotal = GetTickCount();
	DWORD dwStart;
	DWORD dwElapsed;
#define AUFGABE 1
#ifdef AUFGABE
#if AUFGABE == 1
	std::cout << "Aufgabe 1" << endl;
	typedef bg::model::point<double, 2, bg::cs::cartesian> boostPoint;
	typedef std::pair<boostPoint, vertex_t> rtree_value;
	typedef boost::geometry::index::rtree<rtree_value, boost::geometry::index::quadratic<16>> knn_rtree_t_ohne_worm;
	// number of nodes k
	const int k = 1000;
	const int nearestSearch = 1;
	const double stepSize = 0.002;

	std::mt19937 generator = std::mt19937(std::random_device().operator()());
	std::uniform_real_distribution<double> random(0, 1);

	graph_t g;
	knn_rtree_t_ohne_worm rtree;

	Eigen::VectorXd qStart(2);
	qStart << 0.5, 0.5;

	vertex_prop_t startVertex;
	startVertex.q_ = qStart;
	vertex_t startVertexIndex = boost::add_vertex(startVertex, g);
	boostPoint bpStart = boostPoint(qStart[0], qStart[1]);
	rtree.insert(rtree_value(bpStart, startVertexIndex));

	dwStart = GetTickCount();
	std::cout << "Exploring..." << endl;
	for (int i = 1; i <= k; i++) {
		double rand = random(generator);
		Eigen::VectorXd qNext(2);
		qNext << random(generator), random(generator);
		vertex_prop_t nextVertex;
		nextVertex.q_ = qNext;
		vertex_t nextVertexIndex = boost::add_vertex(nextVertex, g);


		std::vector<rtree_value> nearest;
		boostPoint test = boostPoint(qNext[0], qNext[1]);
		rtree.query(bgi::nearest(test, nearestSearch), std::back_inserter(nearest));
		for (int j = 0; j < nearestSearch; j++) {
			Eigen::VectorXd nearVect(2);
			nearVect << nearest[j].first.get<0>(), nearest[j].first.get<1>();
			Eigen::VectorXd heading = (qNext - nearVect).normalized();

			Eigen::VectorXd act = nearVect;
			vertex_t actIndex = nearest[j].second;
			// add intermediates
			while (act != qNext) {
				Eigen::VectorXd n;
				if ((qNext - act).norm() > stepSize) {
					n = act + (heading * stepSize);
				}
				else {
					n = qNext;
				}
				vertex_prop_t nprop;
				nprop.q_ = n;
				vertex_t nIndex = boost::add_vertex(nprop, g);

				// add to rtree
				boostPoint bpn = boostPoint(n[0], n[1]);
				rtree.insert(rtree_value(bpn, nIndex));

				// add edge
				boost::add_edge(actIndex, nIndex, g);

				act = n;
				actIndex = nIndex;
			}
		}
		dwElapsed = GetTickCount() - dwStart;
		std::cout << "\r" << i << "/" << k << " (" << (dwElapsed / i * (k - i) / 1000) << "s remaining)";
	}
	std::cout << endl;
	dwElapsed = GetTickCount() - dwStart;
	std::cout << "took " << dwElapsed << " ms\n\n";

	write_gnuplot_file2(g, "graph.dat");
#else
	std::cout << "!Aufgabe 1" << endl;

	WormCell cell;
	Eigen::VectorXd qStart(5), qGoal(5), q(5);
	vector<Eigen::VectorXd> path; // create a point vector for storing the path
	graph_t g;
	knn_rtree_t rtree;
	const float stepsize = .025f;

	const int nNodes = 25000;
#define TEST_CASE 0
#ifdef TEST_CASE
#if TEST_CASE == 0
	// Example
	std::cout << "Example" << endl;
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
	std::cout << "Test case 1" << endl;
	qStart << .6, .1, 0., 0., 0.;
	qGoal << .1, .8, DEG2RAD(-90.f), 0., 0.;
#elif TEST_CASE == 2
	std::cout << "Test case 2" << endl;
	qStart << .1, .8, DEG2RAD(-90.f), DEG2RAD(-180.f), DEG2RAD(180.f);
	qGoal << .9, .4, DEG2RAD(-90.f), DEG2RAD(-180.f), DEG2RAD(180.f);
#elif TEST_CASE == 3
	std::cout << "Test case 3" << endl;
	qStart << .9, .4, DEG2RAD(-90.f), DEG2RAD(-180.f), DEG2RAD(180.f);
	qGoal << .9, .75, DEG2RAD(-180.f), 0., 0.;
#elif TEST_CASE == 4
	std::cout << "Test case 4" << endl;
	qStart << .9, .75, DEG2RAD(-180.f), 0., 0.;
	qGoal << .5, .45, DEG2RAD(-180.f), 0., 0.;
#elif TEST_CASE == 5
	std::cout << "Test case 5" << endl;
	qStart << .5, .45, DEG2RAD(-180.f), 0., 0.;
	qGoal << .6, .95, DEG2RAD(-90.f), 0., 0.;
#elif TEST_CASE == 6
	std::cout << "Test case 6" << endl;
	qStart << .5, .45, DEG2RAD(-180.f), 0., 0.;
	qGoal << .6, .95, DEG2RAD(-90.f), 0., 0.;
#elif TEST_CASE == 7
	std::cout << "Test case 7 / colliding goal" << endl;
	qStart << .6, .95, DEG2RAD(-90.f), 0., 0.;
	qGoal << .7, .95, DEG2RAD(-90.f), 0., 0.;
#elif TEST_CASE == 8
	std::cout << "Test case 8 / colliding start" << endl;
	qStart << .7, .95, DEG2RAD(-90.f), 0., 0.;
	qGoal << .6, .95, DEG2RAD(-90.f), 0., 0.;
#elif TEST_CASE == 9
	std::cout << "Test case 9 / unreachable goal" << endl;
	qStart << .6, .95, DEG2RAD(-90.f), 0., 0.;
	qGoal << .6, 1.05, DEG2RAD(-90.f), 0., 0.;
#elif TEST_CASE == 10
	std::cout << "Test case 10 / unreachable start" << endl;
	qStart << .6, 1.05, DEG2RAD(-90.f), 0., 0.;
	qGoal << .6, .95, DEG2RAD(-90.f), 0., 0.;
#endif
#endif

	g = graph_t(0);
	int connectedNodes = 0;
	bool resampling = false;
	int resamplesDone = 0;
	bool solutionFound = false;

	vertex_t startIndex;
	vertex_t goalIndex;

	if (!cell.CheckPosition(qStart)){
		cout << "Startposition ungueltig!" << endl;
		return 0;
	}
	if (!cell.CheckPosition(qGoal)){
		cout << "Endposition ungueltig!" << endl;
		return 0;
	}


	// Startzeit
	resamplesDone++;
	dwStart = GetTickCount();
	// 1. step: building up a graph g consisting of nNodes vertices
	std::cout << "1. Step: building " << nNodes << " nodes for the RRT-graph" << endl;

	for (int index = 0; index < nNodes; index++) {

	}

	// Zeit ausgeben ( in ms )
	dwElapsed = GetTickCount() - dwStart;
	std::cout << "took " << dwElapsed << " ms\n\n";

	// ###################

	// Startzeit
	dwStart = GetTickCount();
	// 3. Step: connecting start configuration to graph
	std::cout << "2. Step: connecting start configuration to graph" << endl;



	// Zeit ausgeben ( in ms )
	dwElapsed = GetTickCount() - dwStart;
	std::cout << "took " << dwElapsed << " ms\n\n";

	// ###################

	// Startzeit
	dwStart = GetTickCount();
	// 4. Step: connecting goal configuration to graph
	std::cout << "3. Step: connecting goal configuration to graph" << endl;



	// Zeit ausgeben ( in ms )
	dwElapsed = GetTickCount() - dwStart;
	std::cout << "took " << dwElapsed << " ms\n\n";

	// ###################


	// ###################

	// Startzeit
	dwStart = GetTickCount();
	// 5. Step: searching for shortest path
	std::cout << "4. Step: searching for shortest path" << endl;

	std::vector<vertex_t> p(num_vertices(g));
	std::vector<float> d(num_vertices(g));

	boost::dijkstra_shortest_paths(g, startIndex,
		predecessor_map(boost::make_iterator_property_map(p.begin(), get(boost::vertex_index, g))).
		distance_map(boost::make_iterator_property_map(d.begin(), get(boost::vertex_index, g))));

	std::cout << "distance to goal = " << d[goalIndex] << endl;

	// Zeit ausgeben ( in ms )
	dwElapsed = GetTickCount() - dwStart;
	std::cout << "took " << dwElapsed << " ms\n\n";

	// ###################

	// Startzeit
	dwStart = GetTickCount();
	// 6. Step: building easyrob path
	std::cout << "5. Step: building easyrob path" << endl;

	// Zeit ausgeben ( in ms )
	dwElapsed = GetTickCount() - dwStart;
	std::cout << "took " << dwElapsed << " ms\n\n";

	// ###################

	return EXIT_SUCCESS;
#endif
#endif // AUFGABE

	// Zeit ausgeben ( in ms )
	dwElapsed = GetTickCount() - dwStartTotal;
	std::cout << "total time took " << dwElapsed << " ms\n\n";
}

