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

#define SOLUTION

using namespace std;

vector<Point> VisibilityGraph(Graph g, const int nHind)
{
    typedef boost::graph_traits<Graph>::vertex_descriptor vertex_descriptor;
    typedef boost::graph_traits<Graph>::edge_descriptor edge_descriptor;
    typedef std::pair<int, int> Edge;

    vector<Point> path; // create a point vector for storing the path

    // Example for access to the coordinates of the vertices
    for (int i = 0; i < nHind * 4; i++)
	{
		//boost::add_edge(i, i + 1, i, g);
        cout << g[i].pt.x << "x " << g[i].pt.y << endl;
	}

#ifdef SOLUTION

	// Draw Obstacles

	for (int i = 0; i < nHind; i++)
	{
		for (int j = 0; j < 4; j++){
			boost::add_edge(i * 4 + j, i * 4 + (j + 1) % 4, 0, g);
		}
		cout << g[i].pt.x << "x " << g[i].pt.y << endl;
	}

#endif SOLUTION

    write_gnuplot_file(g, "VisibilityGraph.dat");

    return path;
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
