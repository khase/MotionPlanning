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

string stringf(const char* format, ...)
{
	va_list args;
	va_start(args, format);

	int size = _vscprintf(format, args) + 1;
	std::unique_ptr<char[]> buf(new char[size]);

#ifndef _MSC_VER
	vsnprintf(buf.get(), size, format, args);
#else
	vsnprintf_s(buf.get(), size, _TRUNCATE, format, args);
#endif

	va_end(args);

	return string(buf.get());
}

string toString(const Eigen::VectorXd& vect)
{
	return stringf("[%f, %f, %f, %f, %f]", vect[0], vect[1], vect[2], vect[3], vect[4]);
}

Eigen::VectorXd getVector(vertex_t vert, graph_t &g) {
	return g[vert].q_;
}

vertex_t addVertex(Eigen::VectorXd vertex, WormCell &cell, graph_t &g, knn_rtree_t &rtree) {
	vertex_prop_t prop;
	prop.q_ = vertex;
	vertex_t vert = boost::add_vertex(prop, g);

	rtree_value val;
	MyWorm robot = cell.Robot();
	robot.q() = vertex;
	val = rtree_value(robot, vert);
	rtree.insert(val);

	return vert;
}


vertex_t addIntermediates(Eigen::VectorXd from, Eigen::VectorXd to, WormCell &cell, graph_t &g, knn_rtree_t &rtree, double stepsize) {
	vertex_t vert;

	return vert;
}

std::vector<rtree_value> findNearest(Eigen::VectorXd vertex, int count, knn_rtree_t &rtree) {
	Eigen::VectorXd actVector = vertex;
	std::vector<rtree_value> nearest;
	MyWorm test = MyWorm(actVector);
	rtree.query(bgi::nearest(test, count), std::back_inserter(nearest));

	return nearest;
}

Eigen::VectorXd doStep(Eigen::VectorXd vertex, Eigen::VectorXd goal, double stepSize) {
	Eigen::VectorXd heading = goal - vertex;
	if (heading.norm() <= stepSize) {
		// std::cout << toString(vertex) << " -> " << toString(goal) << " check" << endl;
		return goal;
	}
	heading.normalize();
	// std::cout << toString(vertex) << " -> " << toString(vertex + (heading * stepSize)) << endl;
	return vertex + (heading * stepSize);
}

Eigen::VectorXd exploreRandom(graph_t &g, knn_rtree_t &rtree, WormCell &cell, double stepSize, graph_t &gGlobal, knn_rtree_t &rtreeGlobal) {
	// do until on expantion was successful
	while (true) {
		// generate random configuration
		Eigen::VectorXd heading = cell.NextRandomCspace();

		// find nearest config in current graph
		rtree_value val = findNearest(heading, 1, std::ref(rtree))[0];
		Eigen::VectorXd act = getVector(val.second, std::ref(g));

		// do one step towards random config
		Eigen::VectorXd qNew = doStep(act, heading, stepSize);

		// check motion (is possible??)
		if (cell.CheckMotion(act, qNew)) {
			// expand graph (add intermediates)
			double intermediateStepSize = stepSize / 10;
			do {
				Eigen::VectorXd intermediate = doStep(act, qNew, intermediateStepSize);
				float length = (intermediate - act).norm();

				vertex_t vertNew;
				vertNew = addVertex(qNew, std::ref(cell), std::ref(g), std::ref(rtree));
				boost::add_edge(findNearest(act, 1, std::ref(rtree))[0].second, vertNew, length, g);
				// add same vertex/edge to global graph
				vertNew = addVertex(qNew, std::ref(cell), std::ref(gGlobal), std::ref(rtreeGlobal));
				boost::add_edge(findNearest(act, 1, std::ref(rtreeGlobal))[0].second, vertNew, length, gGlobal);

				act = intermediate;
			} while (act.isApprox(qNew, intermediateStepSize / 10));

			// return newly explored config
			return qNew;
		}
	}
}

bool expandTowards(graph_t &g, knn_rtree_t &rtree, WormCell &cell, double stepSize, Eigen::VectorXd target, graph_t &gGlobal, knn_rtree_t &rtreeGlobal) {
	// find nearest existing Vector
	rtree_value val = findNearest(target, 1, std::ref(rtree))[0];
	vertex_t actIndex = val.second;
	Eigen::VectorXd act = getVector(val.second, std::ref(g));

	// do until connected or obstacle is hit
	do {
		// do one step towards the target
		Eigen::VectorXd qNew = doStep(act, target, stepSize);
		float length = (qNew - act).norm();

		// check motion (is possible??)
		if (!cell.CheckMotion(act, qNew)) {
			return false;
		}

		if (qNew.isApprox(target, stepSize / 10)) {
			// reached Target
			rtree_value connectVal = findNearest(target, 1, std::ref(rtree))[0];

			// connect start and goal components in global graph
			boost::add_edge(
				findNearest(act, 1, std::ref(rtreeGlobal))[0].second,
				findNearest(target, 1, std::ref(rtreeGlobal))[0].second,
				length,
				gGlobal);

			return true;
		}
		else {
			// expand graph
			vertex_t vertNew;
			vertNew = addVertex(qNew, std::ref(cell), std::ref(g), std::ref(rtree));
			boost::add_edge(actIndex, vertNew, length, g);
			actIndex = vertNew;

			// add same vertex/edge to global graph
			vertNew = addVertex(qNew, std::ref(cell), std::ref(gGlobal), std::ref(rtreeGlobal));
			boost::add_edge(findNearest(act, 1, rtreeGlobal)[0].second, vertNew, length, gGlobal);

			act = qNew;
		}
	} while (true);
	return false;
}

/***********************************************************************************************************************************/
int _tmain(int argc, _TCHAR* argv[])
{
	DWORD dwStartTotal = GetTickCount();
	DWORD dwStart;
	DWORD dwElapsed;
#define AUFGABE 2
#ifdef AUFGABE
#if AUFGABE == 1
	std::cout << "Aufgabe 1 wird ausgefuehrt" << endl << endl;
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
		// generate random configuration
		Eigen::VectorXd qNext(2);
		qNext << random(generator), random(generator);
		vertex_prop_t nextVertex;
		nextVertex.q_ = qNext;
		// add vertex to graph
		vertex_t nextVertexIndex = boost::add_vertex(nextVertex, g);

		// find nearest neighbour
		std::vector<rtree_value> nearest;
		boostPoint test = boostPoint(qNext[0], qNext[1]);
		rtree.query(bgi::nearest(test, nearestSearch), std::back_inserter(nearest));
		for (int j = 0; j < nearestSearch; j++) {
			// build connection
			Eigen::VectorXd nearVect(2);
			nearVect << nearest[j].first.get<0>(), nearest[j].first.get<1>();
			Eigen::VectorXd heading = (qNext - nearVect).normalized();

			Eigen::VectorXd act = nearVect;
			vertex_t actIndex = nearest[j].second;
			// add intermediates
			while (act != qNext) {
				// step towards the new configuration
				Eigen::VectorXd n;
				if ((qNext - act).norm() > stepSize) {
					n = act + (heading * stepSize);
				}
				else {
					n = qNext;
				}
				// add vertex to graph
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
		std::cout << "\r" << i << "/" << k;
		// save current graph after each 10%
		if (i % (k/10) == 0) {
			std::cout << " (" << (dwElapsed / i * (k - i) / 1000) << "s remaining) ";
			std::stringstream nameBuilder;
			nameBuilder << "graph-" << i / (k/10) << ".dat";
			write_gnuplot_file2(g, nameBuilder.str());
		}
	}
	std::cout << endl;
	dwElapsed = GetTickCount() - dwStart;
	std::cout << "took " << dwElapsed << " ms\n\n";

	write_gnuplot_file2(g, "graph.dat");
#else
	std::cout << "Aufgabe 2 wird ausgefuehrt" << endl << endl;

	WormCell cell;
	Eigen::VectorXd qStart(5), qGoal(5), q(5);
	vector<Eigen::VectorXd> path; // create a point vector for storing the path
	graph_t g, gStart, gGoal;
	knn_rtree_t rtree, rtreeStart, rtreeGoal;
	const float stepsize = .025f;
	const float graphstepsize = .1f;

	const int nNodes = 25000;
#define TEST_CASE 2
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
	gStart = graph_t(0);
	gGoal = graph_t(0);

	vertex_t startIndex;
	vertex_t goalIndex;

	if (!cell.CheckPosition(qStart)) {
		cout << "Startposition ungueltig!" << endl;
		return 0;
	}
	if (!cell.CheckPosition(qGoal)) {
		cout << "Endposition ungueltig!" << endl;
		return 0;
	}

	if (!MyWorm::IsInsideRange(qStart)) {
		cout << "Startposition ausserhalb des Arbeitsraums!" << endl;
		return 0;
	}

	if (!MyWorm::IsInsideRange(qGoal)) {
		cout << "Endposition ausserhalb des Arbeitsraums!" << endl;
		return 0;
	}


	// Startzeit
	dwStart = GetTickCount();
	// 1. step: building up a graph g consisting of nNodes vertices
	std::cout << "Exploring..." << endl;

	vertex_t startInGlobalGraph = addVertex(qStart, std::ref(cell), std::ref(g), std::ref(rtree));
	vertex_t startInStartGraph = addVertex(qStart, std::ref(cell), std::ref(gStart), std::ref(rtreeStart));

	vertex_t goalInGlobalGraph = addVertex(qGoal, std::ref(cell), std::ref(g), std::ref(rtree));
	vertex_t goalInGoalGraph = addVertex(qGoal, std::ref(cell), std::ref(gGoal), std::ref(rtreeGoal));

	bool connected = false;
	while (!connected) {
		// Magic do not touch
		if (!connected) {
			// random exploring for goal-tree
			Eigen::VectorXd target = exploreRandom(
				std::ref(gGoal),
				std::ref(rtreeGoal),
				std::ref(cell),
				graphstepsize,
				std::ref(g),
				std::ref(rtree));

			// try to connect from start to goal
			connected |= expandTowards(
				std::ref(gStart),
				std::ref(rtreeStart),
				std::ref(cell),
				graphstepsize,
				target,
				std::ref(g),
				std::ref(rtree));
		}
		// Magic do not touch
		if (!connected) {
			// random exploring for start-tree
			Eigen::VectorXd target = exploreRandom(
				std::ref(gStart),
				std::ref(rtreeStart),
				std::ref(cell),
				graphstepsize,
				std::ref(g),
				std::ref(rtree));

			// try to connect from goal to start
			connected |= expandTowards(
				std::ref(gGoal),
				std::ref(rtreeGoal),
				std::ref(cell),
				graphstepsize,
				std::ref(target),
				std::ref(g),
				std::ref(rtree));
		}
		
	}

	// Zeit ausgeben ( in ms )
	dwElapsed = GetTickCount() - dwStart;
	std::cout << "took " << dwElapsed << " ms\n\n";

	// ###################

	std::cout << "Metrics for start graph:" << endl;
	std::cout << "  Vertices:" << gStart.m_vertices.size() << endl;
	std::cout << "  Edges:" << gStart.m_edges.size() << endl;
	std::cout << "Metrics for goal graph:" << endl;
	std::cout << "  Vertices:" << gGoal.m_vertices.size() << endl;
	std::cout << "  Edges:" << gGoal.m_edges.size() << endl;
	std::cout << "Metrics for Global graph:" << endl;
	std::cout << "  Vertices:" << g.m_vertices.size() << endl;
	std::cout << "  Edges:" << g.m_edges.size() << endl;
	std::cout << endl;

	// ###################

	// Startzeit
	dwStart = GetTickCount();
	// 5. Step: searching for shortest path
	std::cout << "4. Step: searching for shortest path" << endl;

	std::vector<int> component(num_vertices(g));

	std::vector<vertex_t> p(num_vertices(g));
	std::vector<float> d(num_vertices(g));
	if (g.m_vertices.size() > 0) {
		int num = connected_components(g, &component[0]);
		// std::cout << num << " Components" << endl;
		if (g.m_vertices.size() > 0 && component[startInGlobalGraph] == component[goalInGlobalGraph]) {

			boost::dijkstra_shortest_paths(g, startInGlobalGraph,
				predecessor_map(boost::make_iterator_property_map(p.begin(), get(boost::vertex_index, g))).
				distance_map(boost::make_iterator_property_map(d.begin(), get(boost::vertex_index, g))));

			std::cout << "distance to goal = " << d[goalInGlobalGraph] << endl;
		}
		else {
			std::cout << "Graph not connected" << endl;
		}
	}
	else {
		std::cout << "Graph has no vertices" << endl;
	}

	// Zeit ausgeben ( in ms )
	dwElapsed = GetTickCount() - dwStart;
	std::cout << "took " << dwElapsed << " ms\n\n";

	// ###################

	// Startzeit
	dwStart = GetTickCount();
	// 6. Step: building easyrob path
	std::cout << "5. Step: building easyrob path" << endl;

	vertex_t current = goalInGlobalGraph;
	while (current != startInGlobalGraph) {
		path.push_back(g[current].q_);
		current = p[current];
	}
	path.push_back(g[startInGlobalGraph].q_);

	write_easyrob_program_file(path, "solution.prg", false);

	// Zeit ausgeben ( in ms )
	dwElapsed = GetTickCount() - dwStart;
	std::cout << "took " << dwElapsed << " ms\n\n";

	// ###################

#endif
#endif // AUFGABE

	// Zeit ausgeben ( in ms )
	dwElapsed = GetTickCount() - dwStartTotal;
	std::cout << "total time took " << dwElapsed << " ms\n\n";

	return EXIT_SUCCESS;
}

