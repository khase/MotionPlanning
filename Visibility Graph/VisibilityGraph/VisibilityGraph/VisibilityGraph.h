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
#include "boost/geometry/algorithms/intersection.hpp"
#include "boost/geometry/geometries/segment.hpp"

// additional Information for each vertex
struct VertexProperty
{
    Point pt; // point coordinates of vertex
};

typedef boost::property<boost::edge_weight_t, float> EdgeWeightProperty;
typedef boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS, VertexProperty, EdgeWeightProperty> Graph;
typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> point_t;
typedef boost::geometry::model::segment<point_t> Segment;
typedef boost::graph_traits<Graph>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<Graph>::edge_descriptor edge_descriptor;
typedef std::pair<int, int> Edge;
typedef boost::graph_traits<Graph>::edge_iterator edge_iter;

std::vector<Point> VisibilityGraph(Graph g, const int nHind);
void write_gnuplot_file(Graph g, std::string filename);

bool intersects(Segment Sight, Graph obstGraph);
bool findPointInVector(point_t p, std::vector<point_t> v);
bool arePointsIdentitcal(point_t p1, point_t p2);

#endif /* __VISIBILITYGRAPH_H__ */
