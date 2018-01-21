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
#include <vector>
#include <algorithm>
#include <utility>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp> // for normal distribution
using namespace std;
namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;
typedef std::pair<Eigen::VectorXd, unsigned> value;
typedef bg::model::point<Eigen::VectorXd, 1, bg::cs::cartesian> point;
typedef std::pair<int, vertex_t> intVertexPair;
std::list<intVertexPair> intVertexList;
std::mutex listMutex;  // protects intVertexList

// addEdges: Fügt Kanten zwischen einer allen Nodes in der gegebenen Spanne (zwischen Start und End im Graph) und resampleCnt*3 Nachbarn dieser Node hinzu.
int addEdges(int num, int start, int end, graph_t &g, knn_rtree_t &rtree, int resampleCnt) {
	WormCell cell;
	for (int index = start; index < end; index++) {
		if (index % 100 == 0) {
			std::cout << ".";
		}
		vertex_t vert = vertex_t(index);												// Vertex erstellen
		Eigen::VectorXd actVector = g[vert].q_;											// Aktuell betrachteten Vektor erstellen
		std::vector<rtree_value> nearest;												// Liste, in welcher nachher die nächsten Nodes gespeichert werden erstellen
		MyWorm test = MyWorm(actVector);												// Worm mit aktueller Node erstellen, um die nächsten Nodes dazu finden zu können
		rtree.query(bgi::nearest(test, resampleCnt*3), std::back_inserter(nearest));	// Gibt die nächsten resampleCnt*3 Nodes zurück
		int cntNoConnectionFound = 0;					// Zähler, wird später beim resampling verwendet

		for (auto &q : nearest)
		{
			Eigen::VectorXd nearestVector = g[q.second].q_;
			if (!boost::edge(q.second, vert, g).second && cell.CheckMotion(nearestVector, actVector)) {		// Checkt, ob die Kante bereits im Knoten ist und ob die Bewegung zwischen dem aktuellen und dem gefundenen nächsten okay ist
				float lengthOfEdge = (nearestVector - actVector).norm();			// Speichert die Länge der Bewegung/Kante
				boost::add_edge(q.second, vert, lengthOfEdge, g);					// Fügt die Kante dem Graphen hinzu
			}
			else if (!cell.CheckMotion(nearestVector, actVector))			// Ist die Bewgung nicht okay, sprich kollidiert der Wurm während der Bewegung, wird der Zähler hochgezählt
			{
				cntNoConnectionFound++;
			}
		}
		if (cntNoConnectionFound == 0)
			continue;
		std::lock_guard<std::mutex> lock(listMutex);		// Lock wird gebruacht wegen Multithreading
		intVertexList.push_back(intVertexPair(cntNoConnectionFound, vert));		// Füge der Liste aller Nodes die Anzahl der fehlgeschlagenen Verbindungsversuche sowie den zugehörigen Vertex hinzu
	}
	return 0;
}


/*
	Erstellt auf Basis der Liste intVertexList Nodes, welche sich hauptsächlich in der Nähe von Obstacles befinden. Dieses Verfahren basiert auf dem Gausschen Sampling.
*/
int resample(int nNodes, graph_t &g, knn_rtree_t &rtree, WormCell &cell, boost::mt19937 &rng)
{
	int index = 0;
	while (intVertexList.size() > 0){						// Arbeite solange bis die Liste leer ist
		Eigen::VectorXd actVector;
		intVertexPair actPair = intVertexList.front();		// Hole das oberste Element der Liste.
		intVertexList.pop_front();							// Lösche das oberste Element aus der Liste
		actVector = g[actPair.second].q_;					// Zwischenspeichern der aktuellen Node
		for (int i = 0; i < (actPair.first); i++)			// So viele Resamples wie fehlgeschlagene Verbindungen
		{
			double newVectorValues[5];
			bool newVectorOkay = true;
			for (int i = 0; i < 5; i++)					// Erstelle für alle 5 Freiheitsgrade einen Wert
			{
				int radius = 0.07;
				if (i >= 2)
					radius = 1;

				boost::normal_distribution<> nd(actVector[i], radius);
				boost::variate_generator < boost::mt19937&,
					boost::normal_distribution<> > var_nor(rng, nd);
				double newValue = var_nor();
				newVectorValues[i] = newValue;
			}
			Eigen::VectorXd newVector(5);
			newVector << newVectorValues[0], newVectorValues[1], newVectorValues[2], newVectorValues[3], newVectorValues[4];		// Erstelle aus den Werten eine Node
			if (!MyWorm::IsInsideRange(newVector) || !cell.CheckPosition(newVector)) {		// Checke, ob die Node kollidiert und im Einheitsquadrat liegt. Wenn nein, verwerfe die Node
				continue;
			}
			vertex_prop_t prop;																// Wenn ja, füge die Node in den rtree ein
			prop.q_ = newVector;
			vertex_t vert = boost::add_vertex(prop, g);
			MyWorm robot = cell.Robot();
			robot.q() = prop.q_;
			rtree_value val = rtree_value(robot, vert);
			index++;
			if (index % 100 == 0)
				std::cout << ".";
			rtree.insert(val);
		}
	}
	return index;
}

// Erstellt nNodes zufällige, kollisionsfreie Nodes im Konfigurationsraum
int buildRandomNodes(int nNodes, graph_t &g, knn_rtree_t &rtree, WormCell &cell){
	int index = 0;
	while (index < nNodes) {
		vertex_prop_t prop;		// erstelle vertex Descriptor
		
		prop.q_ = cell.NextRandomCfree();		// generiere zufällige, kollisionsfreie Konfiguration
		
		vertex_t vert = boost::add_vertex(prop, g);
		
		MyWorm robot = cell.Robot();
		
		robot.q() = prop.q_;
		rtree_value val = rtree_value(robot, vert);		// Trage Konfiguration in kd-Tree ein
		rtree.insert(val);

		index++;
		if (index % (nNodes / 10) == 0)
			std::cout << ".";
	}
	return 0;
}

/***********************************************************************************************************************************/
int _tmain(int argc, _TCHAR* argv[])
{
	WormCell cell;
	Eigen::VectorXd qStart(5), qGoal(5), q(5);
	vector<Eigen::VectorXd> path; // create a point vector for storing the path
	graph_t g;
	knn_rtree_t rtree;
	const float stepsize = .025f;
	boost::mt19937 rng(0);

	const int nNodes = 1000;			// Hier wird definiert, wie viele Random-Nodes am Anfang erstellt werden sollen

#define TEST_CASE 3
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

	DWORD dwStartTotal = GetTickCount();			// Erstelle Timer für die Zeitausgabe
	DWORD dwStart;
	DWORD dwElapsed;

	g = graph_t(0);									// Erstelle einen Graph, in dem alle Nodes und Edges gespeichert sind
	int connectedNodes = 0;							// Erstelle mehrere fürs resampling nötige Varialen
	bool resampling = false;
	int resamplesDone = 0;
	bool solutionFound = false;

	vertex_t startIndex;
	vertex_t goalIndex;

	if (!cell.CheckPosition(qStart)){					// Checke, ob die Start- und Zielkonfigurationen plausibel sind
		cout << "Startposition ungueltig!" << endl;
		return 0;
	}
	if (!cell.CheckPosition(qGoal)){
		cout << "Endposition ungueltig!" << endl;
		return 0;
	}

	if (!MyWorm::IsInsideRange(qStart)){
		cout << "Startposition ausserhalb des Arbeitsraums!" << endl;
	return 0;
	}
		
	if (!MyWorm::IsInsideRange(qGoal)){
		cout << "Endposition ausserhalb des Arbeitsraums!" << endl;
		return 0;
	}


	do {			// Hier startet die eigentliche Berechnung des Weges
		// Startzeit
		resamplesDone++;
		dwStart = GetTickCount();

		if (!resampling) {					// Im ersten Schritt werden nNodes viele zufällige Nodes erstellt
			std::cout << "1. Step: building " << nNodes << " random nodes for the graph" << endl; 
			buildRandomNodes(nNodes, std::ref(g), std::ref(rtree), std::ref(cell));
		}
		else								// Ansonsten werden mit Hilfe der Methode resample neue Nodes erzeugt
		{
			std::cout << "1. Step: Resampling...." << endl;
			int nodesAdded = resample(nNodes, std::ref(g), std::ref(rtree), std::ref(cell), std::ref(rng));
			//buildRandomNodes(nNodes * 0.1, std::ref(g), std::ref(rtree), std::ref(cell));
			std::cout << endl << nodesAdded << " Nodes added by resampling" << endl;
		}

		// Zeit ausgeben ( in ms )
		dwElapsed = GetTickCount() - dwStart;
		std::cout << endl << "took " << dwElapsed << " ms\n\n";		// Gebe die benötigte Zeit aus

		// ###################	

		// Startzeit
		dwStart = GetTickCount();
		int connectedNotesBefore = g.m_edges.size();
		// 2. step: building edges for the graph, if the connection of 2 nodes are in free space
		std::cout << "2. Step: buildung edges for the graph" << endl;
		intVertexList.clear();
		const int numThreads = 8;
		std::vector<std::thread> threads;
		int numVertices = g.m_vertices.size();
		for (int i = 0; i < numThreads; ++i) {		// Erstelle numThreads Threads, welche mit Hilfe der Methode addEdges versuchen, die vorhandenen Nodes zu verbinden
			threads.push_back(std::thread(addEdges, i, (i* (numVertices / numThreads)), ((i + 1) * (numVertices / numThreads)), std::ref(g), std::ref(rtree), resamplesDone));
		}
		for (auto& t : threads) {
			t.join();
		}
		connectedNodes = g.m_edges.size() - connectedNotesBefore;
		std::cout << endl << connectedNodes << " edges added" << endl;		// Gebe die Anzahl an erstellten Kanten aus

		// Zeit ausgeben ( in ms )
		dwElapsed = GetTickCount() - dwStart;
		std::cout << "took " << dwElapsed << " ms\n\n";

		// ###################

		// Startzeit
		dwStart = GetTickCount();
		// 3. Step: connecting start configuration to graph
		std::cout << "3. Step: connecting start configuration to graph" << endl;

		vertex_prop_t prop; 
		rtree_value val;
		if (!resampling) {		// Füge im ersten Schritt die Startkonfiguration zum Graphen hinzu
			prop.q_ = qStart;
			startIndex = boost::add_vertex(prop, g);
			val = rtree_value(cell.Robot(), startIndex);
			// Trage Konfiguration in kd-Tree ein
			rtree.insert(val);
		}

		Eigen::VectorXd actVector = g[startIndex].q_;
		std::vector<rtree_value> nearest;
		MyWorm test = MyWorm(actVector);
		rtree.query(bgi::nearest(test, 500), std::back_inserter(nearest));		// Suche die 500 nächsten Nodes und versuche, die Startkonfiguration mit diesen zu verbinden

		int edges = 0;
		for (auto &q : nearest)
		{
			Eigen::VectorXd nearestVector = g[q.second].q_;
			if (!boost::edge(q.second, startIndex, g).second && cell.CheckMotion(nearestVector, actVector)) {
				float lengthOfEdge = (nearestVector - actVector).norm();
				boost::add_edge(q.second, startIndex, lengthOfEdge, g);
				edges++;
			}
		}
		std::cout << edges << " Edges added" << endl;

		// Zeit ausgeben ( in ms )
		dwElapsed = GetTickCount() - dwStart;
		std::cout << "took " << dwElapsed << " ms\n\n";

		// ###################

		// Startzeit
		dwStart = GetTickCount();
		// 4. Step: connecting goal configuration to graph
		std::cout << "4. Step: connecting goal configuration to graph" << endl;

		if (!resampling) {			// Siehe Startkonfiguration
			prop.q_ = qGoal;
			goalIndex = boost::add_vertex(prop, g);
			val = rtree_value(cell.Robot(), goalIndex);
			// Trage Konfiguration in kd-Tree ein
			rtree.insert(val);
		}

		actVector = g[goalIndex].q_;
		nearest;
		test = MyWorm(actVector);
		rtree.query(bgi::nearest(test, 500), std::back_inserter(nearest));

		edges = 0;
		for (auto &q : nearest)
		{
			Eigen::VectorXd nearestVector = g[q.second].q_;
			if (!boost::edge(q.second, goalIndex, g).second && cell.CheckMotion(nearestVector, actVector)) {
				float lengthOfEdge = (nearestVector - actVector).norm();
				boost::add_edge(q.second, goalIndex, lengthOfEdge, g);
				edges++;
			}
		}
		std::cout << edges << " Edges added" << endl;

		// Zeit ausgeben ( in ms )
		dwElapsed = GetTickCount() - dwStart;
		std::cout << "took " << dwElapsed << " ms\n\n";

		// ###################		Gib einige Informationen zum Graphen aus

		std::vector<int> component(num_vertices(g));
		std::cout << g.m_vertices.size() << " Vertices" << endl;
		std::cout << g.m_edges.size() << " Edges" << endl;
		int num = connected_components(g, &component[0]);
		std::cout << num << " Components" << endl;
		if (component[startIndex] != component[goalIndex]) {		// Wenn Start- und Zielkonfiguration nicht im gleichen Graphen liegen muss geresamplt werden
			std::cout << "No Connection!" << endl;
			std::cout << resamplesDone << ". resample..."  << endl;
			resampling = true;
		}
		else {														// Ansonsten wird aus der do-while-Schleife gesprungen
			resampling = false;
			solutionFound = true;
			std::cout << "Solution found after " << resamplesDone-1 << " resample(s)." << endl;
		}

		if (resamplesDone >= 20 && !solutionFound){					// Nach 20 Resamples wird gefragt, ob weitergemacht werden soll, um ein endloses laufen des Programms zu verhindern
			cout << " Already " << resamplesDone << " resamples were processed but there ist still no solution.\r\n  Would you like to continue?\r\n  y / n  ";
			string shouldResample;
			std::cin >> shouldResample;
			if (shouldResample == "y"){
				resampling = true;
			}
			else 
			{ 
				resampling = false; 
				solutionFound = false;
			}
		}

		std::cout << endl;

	} while (resampling);

	if (!solutionFound){
		std::cout << "No solution found. Exiting" << endl;
		return EXIT_FAILURE;
	}

	// ###################

	// Startzeit
	dwStart = GetTickCount();
	// 5. Step: searching for shortest path
	std::cout << "5. Step: searching for shortest path" << endl;

	std::vector<vertex_t> p(num_vertices(g));
	std::vector<float> d(num_vertices(g));

	boost::dijkstra_shortest_paths(g, startIndex,			// Suche mit Hilfe des Dijkstara den kürzesten Pfad zwischen Start- und Zielkonfiguration
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
	std::cout << "6. Step: building easyrob path" << endl;

	vertex_t current = goalIndex;
	while (current != startIndex) {					// Laufe rückwärts durch den vom Dijkstra gefundenen Weg und speichere diesen in der Variable path
		path.push_back(g[current].q_);
		current = p[current];
	}
	path.push_back(g[startIndex].q_);

	write_easyrob_program_file(path, "solution.prg", false);		// Schreibe den Pfad in ein Easyrob-Program-File

	// Zeit ausgeben ( in ms )
	dwElapsed = GetTickCount() - dwStart;
	std::cout << "took " << dwElapsed << " ms\n\n";

	// ###################

	// Zeit ausgeben ( in ms )
	dwElapsed = GetTickCount() - dwStartTotal;
	std::cout << "total time took " << dwElapsed << " ms\n\n";		// Fertig, gib die Zeit aus die für alles benötigt wurde

	return EXIT_SUCCESS;
}

