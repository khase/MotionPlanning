#include "Linie.h"

Linie::Linie()
{
}

Linie::Linie(Point anfang, Point ende)
{
	this->anfang = anfang;
	this->ende = ende;
}


Linie::~Linie()
{
}
int Linie::orientierungZurLinie(Point test)
{
	double val = (this->ende.y - this->anfang.y) * (test.x - this->ende.x) -
		(this->ende.x - this->anfang.x) * (test.y - this->ende.y);

	if (val == 0) return 0;  // Auf einer linie

								// t e	|   e	|
	return (val > 0) ? 1 : 2;	// a	| a t	| rechts oder links von der linie
}

bool Linie::schneidetLinie(Linie test)
{
	if (this->compare(test) || this->extends(test)) {
		return false;
	}

	// alle orientierungen
	int o1a2 = this->orientierungZurLinie(test.anfang);
	int o1e2 = this->orientierungZurLinie(test.ende);
	int o2a1 = test.orientierungZurLinie(this->anfang);
	int o2e1 = test.orientierungZurLinie(this->ende);

	// orientierung der anfangs und endpunkte einer linie (zur anderen Linie) sind unterschiedlich
	if (o1a2 != o1e2 && o2a1 != o2e1)
		return true;

	// keine überschneidung
	return false;
}

bool Linie::compare(Linie test)
{
	// linien sind gleich
	return (
			this->anfang.x == test.anfang.x && this->anfang.y == test.anfang.y	//this->anfang == test.anfang
		&&
			this->ende.x == test.ende.x && this->ende.y == test.ende.y			//this->ende == test.ende
		) || (
			this->anfang.x == test.ende.x && this->anfang.y == test.ende.y		//this->anfang == test.ende 
		&&
			this->ende.x == test.anfang.x && this->ende.y == test.anfang.y		//this->ende == test.anfang
		);
}

bool Linie::extends(Linie test)
{
	// linien verlängern sich / treffen sich an einem anfangs bzw endpunkt
	return (
			this->anfang.x == test.anfang.x && this->anfang.y == test.anfang.y	//this->anfang == test.anfang
		) || (
			this->ende.x == test.ende.x && this->ende.y == test.ende.y			//this->ende == test.ende
		) || (
			this->anfang.x == test.ende.x && this->anfang.y == test.ende.y		//this->anfang == test.ende 
		) || (
			this->ende.x == test.anfang.x && this->ende.y == test.anfang.y		//this->ende == test.anfang
		);
}

int Linie::getObstOrientierung(Obstacle obst)
{
	int orient = 0;
	for (std::vector<Linie>::iterator it = obst.faces.begin(); it != obst.faces.end(); ++it) {
		int oa = this->orientierungZurLinie((*it).anfang);
		int ob = this->orientierungZurLinie((*it).ende);

		if (oa == 0) {
			if (orient == 0 || ob == orient) {
				orient = ob;
			}
			else {
				return 0;
			}
		}
		else if (ob == 0) {
			if (orient == 0 || oa == orient) {
				orient = oa;
			}
			else {
				return 0;
			}
		}
		else if (orient != 0 && (oa != orient || ob != orient)) {
			return 0;
		}
		else {
			orient = oa;
		}
	}
	// 0 == linie geht durch das object
	return orient;
}
bool Linie::isSeparating(Obstacle obst1, Obstacle obst2)
{
	int o1 = this->getObstOrientierung(obst1);
	int o2 = this->getObstOrientierung(obst2);

	bool gehtDurchObst1 = o1 == 0;
	bool gehtDurchObst2 = o2 == 0;
	bool sindGleicheSeite = o1 == o2;

	return !gehtDurchObst1 && !gehtDurchObst2 && !sindGleicheSeite;
}

bool Linie::isSupporting(Obstacle obst1, Obstacle obst2)
{
	int o1 = this->getObstOrientierung(obst1);
	int o2 = this->getObstOrientierung(obst2);

	bool gehtDurchObst1 = o1 == 0;
	bool gehtDurchObst2 = o2 == 0;
	bool sindGleicheSeite = o1 == o2;

	return !gehtDurchObst1 && !gehtDurchObst2 && sindGleicheSeite;
}

double Linie::length()
{
	double distX = std::abs(this->anfang.x - this->ende.x);
	double distY = std::abs(this->anfang.y - this->ende.y);

	return std::sqrt(std::pow(distX, 2) + std::pow(distY, 2));
}

