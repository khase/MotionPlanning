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
#include "boost/graph/graph_traits.hpp"
#include "boost/graph/dijkstra_shortest_paths.hpp"
#include "boost/graph/adjacency_list.hpp"
#include "Point.h"

// additional Information for each vertex
struct VertexProperty
{
    Point pt; // point coordinates of vertex
};

typedef boost::property<boost::edge_weight_t, float> EdgeWeightProperty;
typedef boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS, VertexProperty, EdgeWeightProperty> Graph;

std::vector<Point> VisibilityGraph(Graph g, const int nHind);
void write_gnuplot_file(Graph g, std::string filename);

#endif /* __VISIBILITYGRAPH_H__ */
