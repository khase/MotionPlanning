/******************************************************************************
    file:      VisibilityGraph.cpp
    created:   2016-10-23
    author:    Thomas Horsch

    description: it is a brute force algorithm O(n^3), testing the visibility
    of each pair of edges
******************************************************************************/

#include <iostream>
#include <fstream>
#include "VisibilityGraph.h"
#include <algorithm>
#include <vector>



#define SOLUTION
#define DRAWOBSTACLES

using namespace std;

vector<Point> VisibilityGraph(Graph g, const int nHind)
{


    vector<Point> path; // create a point vector for storing the path

    // Example for access to the coordinates of the vertices
    /*for (int i = 0; i < nHind * 4; i++)
	{
		//boost::add_edge(i, i + 1, i, g);
        cout << g[i].pt.x << "x " << g[i].pt.y << endl;
	}*/

#ifdef SOLUTION

#ifdef DRAWOBSTACLES
	// Draw Obstacles
	Graph obstGraph = Graph(g);
	for (int i = 0; i < nHind; i++)
	{
		cout <<"Drawing Obstacle " << i+1  << endl;
		for (int j = 0; j <= nHind; j++){
			boost::add_edge(i * 4 + j, i * 4 + (j + 1) % 4, 0, obstGraph);
		}
	}
	/*for (int i = 0; i < nHind; i++)
	{
		cout <<"Drawing Obstacle " << i+1  << endl;
		for (int j = 0; j <= nHind; j++){
			for (int k = j; k <= nHind; k++){
				boost::add_edge(i * 4 + j, i * 4 + k % 4, 0, obstGraph);
			}
		}
	}*/
#endif DRAWOBSTACLES
	
	Point start = g[nHind * 4].pt;

	for (int i = 0; i < nHind * 4 + 2; i++)
	{
		for (int j = i; j < nHind * 4 +2; j++)
		{
			std::cout << i << " " << j << " ";
			point_t PointI = point_t(g[i].pt.x, g[i].pt.y);
			point_t PointJ = point_t(g[j].pt.x, g[j].pt.y);
			Segment Sight(PointI, PointJ);

			if (!intersects(Sight, obstGraph))
			{
				boost::add_edge(i, j, 0, g);
			}
		}
	}

#endif SOLUTION

	write_gnuplot_file(g, "VisibilityGraph.dat");

    return path;
}


bool intersects(Segment Sight, Graph obstGraph){
	std::pair<edge_iter, edge_iter> ep;
	edge_iter ei, ei_end;
	for (tie(ei, ei_end) = edges(obstGraph); ei != ei_end; ++ei)
	{
		point_t source = point_t(obstGraph[ei->m_source].pt.x, obstGraph[ei->m_source].pt.y);
		point_t target = point_t(obstGraph[ei->m_target].pt.x, obstGraph[ei->m_target].pt.y);
		std::vector<point_t> output;
		Segment obstLine(source, target);

		if ((arePointsIdentitcal(Sight.first, source) && arePointsIdentitcal(Sight.second, target)) || (arePointsIdentitcal(Sight.first, target) && arePointsIdentitcal(Sight.second, source))){
			return false;
		}

		boost::geometry::intersection(Sight, obstLine, output);
		if (output.size() > 0)
		{
			int intersections = output.size();
			if (findPointInVector(source, output)){
				intersections--;
			}
			if (findPointInVector(target, output)){
				intersections--;
			}
			if (intersections > 0){
				return true;
			}
		}
	}
	return false;
}

bool findPointInVector(point_t p, std::vector<point_t> v)
{
	for (std::vector<point_t>::iterator it = v.begin(); it != v.end(); ++it) {
		if (arePointsIdentitcal((*it), p))
		{
			return true;
		}
	}
	return false;
}

bool arePointsIdentitcal(point_t p1, point_t p2)
{
	return (p1.get<0>() == p2.get<0>() && p1.get<1>() == p2.get<1>());
}


/**************************************************************************/
// Ausgabe einer Plotdatei für gnuplot:
// Aufruf in gnuplot: plot 'visibilitygraph.data' using 1:2 with lines
void write_gnuplot_file(Graph g, string filename)
{
    ofstream myfile;
    myfile.open(filename);

    // Iterate through the edges and print them out
    typedef boost::graph_traits<Graph>::edge_iterator edge_iter;
    std::pair<edge_iter, edge_iter> ep;
    edge_iter ei, ei_end;

    int cnt = 0; // edge counter

    for (tie(ei, ei_end) = edges(g); ei != ei_end; ++ei)
    {
        myfile << g[ei->m_source].pt.x << " " << g[ei->m_source].pt.y << endl;
        myfile << g[ei->m_target].pt.x << " " << g[ei->m_target].pt.y << endl << endl;
        cnt++;
    }

    cout << "Number of edges: " << cnt <<  endl;
    myfile.close();
}
