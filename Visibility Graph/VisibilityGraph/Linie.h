#pragma once

#include <vector>
#include "Point.h"

class Linie
{
public:
	struct Obstacle {
		std::vector<Linie> faces;
	};

	Point anfang;
	Point ende;

	Linie();
	Linie(Point, Point);
	~Linie();

	bool istPunktAufLinie(Point);
	int orientierungZurLinie(Point);
	bool schneidetLinie(Linie);
	
	bool compare(Linie);
	bool extends(Linie);

	bool isSeparating(Obstacle, Obstacle);
	bool isSupporting(Obstacle, Obstacle);

	double length();

private:
	int getObstOrientierung(Obstacle);
};

