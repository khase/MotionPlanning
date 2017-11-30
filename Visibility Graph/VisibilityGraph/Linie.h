#pragma once

#include "Point.h"
class Linie
{
public:
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

	double length();
};

