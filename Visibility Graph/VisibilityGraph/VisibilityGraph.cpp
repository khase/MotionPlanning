/******************************************************************************
    file:      VisibilityGraph.cpp
    created:   2016-10-23
    author:    Thomas Horsch

    description: it is a brute force algorithm O(n^3), testing the visibility
    of each pair of edges
******************************************************************************/

#include <iostream>
#include <iomanip>
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
	int knoten = g.m_vertices.size();

	// erstelle Nachbarschaftsmatrix
	double** adjaMap = new double*[knoten];
	for (int i = 0; i < knoten; i++) {
		adjaMap[i] = new double[knoten];
		for (int j = 0; j < knoten; j++) {
			adjaMap[i][j] = -1;
		}
	}

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

	// alle möglichen verbindungen Testen ob sie mit einem Hinderniss Kollidieren
	for (int i = 0; i < knoten - 1; i++) {
		for (int j = i + 1; j < knoten; j++) {
			Point punktA = g[i].pt;
			Point punktB = g[j].pt;
			Linie linie = Linie(punktA, punktB);

			// gegen alle hindernisse testen
			if (!isIllegal(linie, illegalLines) && isVisible(punktA, punktB, obstLines)) {
				// verbindung ist möglich -> eintragen;
				boost::add_edge(i, j, 0, g);
				adjaMap[i][j] = linie.length();
				adjaMap[j][i] = linie.length();
			}
		}
	}

	printMatrix(adjaMap, knoten, knoten);

	vector<int> weg = dijkstra(adjaMap, knoten - 2, knoten - 1, knoten);

	cout << "Ueber: ";
	for (vector<int>::iterator it = weg.begin(); it != weg.end(); ++it) {
		cout << (*it);
		if (it + 1 != weg.end()) {
			cout << " -> ";
		}

		// koordinaten der Wegpunkte aus dem Grafen extrahieren
		Point point = g[(*it)].pt;
		path.push_back(point);
	}
	cout << endl;

#endif SOLUTION

    write_gnuplot_file(g, "VisibilityGraph.dat");

    return path;
}

void printMatrix(double** Matrix, int sizeX, int sizeY) {
	const int fillWidth = 2;
	const char fillChar = ' ';
	const char lineChar = '-';

	const int lineWidth = (sizeX + 1) * (fillWidth + 2);

	cout << setw(fillWidth) << " " << " |";
	for (int j = 0; j < sizeY; j++) {
		cout << setw(fillWidth) << j << " |";
	}
	cout << endl;
	cout << setw(lineWidth) << setfill(lineChar) << "" << setfill(fillChar) << endl;

	for (int i = 0; i < sizeX; i++) {
		cout << setw(fillWidth) << i << " |";
		for (int j = 0; j < sizeY; j++) {
			cout << setw(fillWidth) << ((Matrix[i][j] >= 0) ? "->" : "") << " |";
		}
		cout << endl;
		cout << setw(lineWidth) << setfill(lineChar) << "" << setfill(fillChar) << endl;
	}
}

vector<int> dijkstra(double** nachbarn, int start, int ziel, int knoten)
{
	vector<Wegpunkt> besucht;
	vector<Wegpunkt> moeglicheWege;

	// Start festsetzen
	Wegpunkt a;
	a.nummer = start;
	a.weg.push_back(start);
	a.weglänge = 0;

	moeglicheWege.push_back(a);

	while (!moeglicheWege.empty()) {
		// sortieren
		std::sort(moeglicheWege.begin(), moeglicheWege.end(), a);

		// nächsten kürzesten weg gehen
		Wegpunkt next = moeglicheWege[moeglicheWege.size() - 1];
		moeglicheWege.pop_back();

		// prüfen ob wir am Ziel sind
		if (next.nummer == ziel) {
			cout << "Weg gefunden, totale länge: " << next.weglänge << endl;
			// gelaufene sträcke zurück geben
			return next.weg;
		}

		// prüfen ob wir schonmal hier waren (dann mit einem garantiert kürzeren weg)
		bool warSchonDa = false;
		for (vector<Wegpunkt>::iterator it = besucht.begin(); it != besucht.end(); ++it) {
			if ((*it).nummer == next.nummer) {
				warSchonDa = true;
				break;
			}
		}

		if (!warSchonDa) {
			// wir waren noch nie hier -> also weg als kürzesten weg aufnehmen
			besucht.push_back(next);

			// alle von hieraus möglichen wege in die liste "möglicheWege" aufnehmen
			for (int i = 0; i < knoten; i++) {
				if (nachbarn[next.nummer][i] >= 0) {
					Wegpunkt neu;
					neu.nummer = i;
					neu.weg = next.weg;
					neu.weg.push_back(i);
					neu.weglänge = next.weglänge + nachbarn[next.nummer][i];

					moeglicheWege.push_back(neu);
				}
			}
		}

	}
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
