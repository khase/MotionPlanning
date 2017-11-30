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
    //for (int i = 0; i < nHind * 4 + 2; i++)
    //{
    //    cout << g[i].pt.x << " " << g[i].pt.y << endl;
    //}

#ifdef SOLUTION
	std::vector<Linie> obstLines;
	std::vector<Linie> illegalLines;	// inside an obstacle
	for (int i = 0; i < nHind; i++)
	{
		// Hindernissdiagonalen
		for (int j = 0; j < 2; j++) {
			int indexA = i * 4 + j;
			int indexB = i * 4 + (j + 2);
			//boost::add_edge(indexA, indexB, 0, g);
			Point punktA = g[indexA].pt;
			Point punktB = g[indexB].pt;
			Linie linie = Linie(punktA, punktB);
			illegalLines.push_back(linie);
		}

		// Hindernisswände
		for (int j = 0; j < 4; j++) {
			int indexA = i * 4 + j;
			int indexB = i * 4 + (j + 1) % 4;
			//boost::add_edge(indexA, indexB, 0, g);
			Point punktA = g[indexA].pt;
			Point punktB = g[indexB].pt;
			Linie linie = Linie(punktA, punktB);
			obstLines.push_back(linie);
		}
	}

	int knoten = g.m_vertices.size();

	// alle möglichen verbindungen Testen ob sie mit einem Hinderniss Kollidieren
	for (int i = 0; i < knoten - 1; i++) {
		for (int j = i + 1; j < knoten; j++) {
			Point punktA = g[i].pt;
			Point punktB = g[j].pt;
			Linie linie = Linie(punktA, punktB);

			// gegen alle hindernisse testen
			if (!isIllegal(linie, illegalLines) && isVisible(punktA, punktB, obstLines)) {
				boost::add_edge(i, j, 0, g);
			}
		}
	}

	cout << knoten << endl;

#endif SOLUTION

    write_gnuplot_file(g, "VisibilityGraph.dat");

    return path;
}
bool isVisible(Point testA, Point testB, std::vector<Linie> obstLines) {
	Linie linie = Linie(testA, testB);

	for (int i = 0; i < obstLines.size(); i++) {
		if (linie.schneidetLinie(obstLines[i])) {
			return false;
		}
	}
	return true;
}

bool isIllegal(Linie test, std::vector<Linie> illegalLines) {
	for (int i = 0; i < illegalLines.size(); i++) {
		if (test.compare(illegalLines[i])) {
			return true;
		}
	}
	return false;
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
