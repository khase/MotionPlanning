/******************************************************************************
file:      VisibilityGraph.h
created:   2016-10-23
author:    Thomas Horsch

description: it is a brute force algorithm O(n^3), testing the visibility
of each pair of edges
******************************************************************************/
#ifndef __VISIBILITYGRAPH_H__
#define __VISIBILITYGRAPH_H__

#include <vector>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/adjacency_list.hpp>
#include "Point.h"
#include "Linie.h"

// additional Information for each vertex
struct VertexProperty
{
    Point pt; // point coordinates of vertex
};

struct Wegpunkt {
	int nummer;
	std::vector<int> weg;
	double weglänge;

	bool operator() (Wegpunkt i, Wegpunkt j) { return (i.weglänge > j.weglänge); }
};

typedef boost::property<boost::edge_weight_t, float> EdgeWeightProperty;
typedef boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS, VertexProperty, EdgeWeightProperty> Graph;

std::vector<Point> VisibilityGraph(Graph g, const int nHind);
void write_gnuplot_file(Graph g, std::string filename);

bool isVisible(Point, Point, std::vector<Linie>);
bool isIllegal(Linie, std::vector<Linie>);

void printMatrix(double**, int, int);

std::vector<int> dijkstra(double**, int, int, int);

#endif /* __VISIBILITYGRAPH_H__ */
