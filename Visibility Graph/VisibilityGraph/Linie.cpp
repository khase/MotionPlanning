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

bool Linie::istPunktAufLinie(Point test)
{
	return this->ende.x <= std::max(this->anfang.x, test.x) && this->ende.x >= std::min(this->anfang.x, test.x) &&
		this->ende.y <= std::max(this->anfang.y, test.y) && this->ende.y >= std::min(this->anfang.y, test.y);
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

	// ein anfangs/endpunkt einer linie liegt genau auf der anderen linie
	/*if (o1a2 == 0 && this->istPunktAufLinie(test.anfang))
		return true;
	if (o1e2 == 0 && this->istPunktAufLinie(test.ende))
		return true;
	if (o2a1 == 0 && test.istPunktAufLinie(this->anfang))
		return true;
	if (o2e1 == 0 && test.istPunktAufLinie(this->ende))
		return true;*/

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

double Linie::length()
{
	double distX = std::abs(this->anfang.x - this->ende.x);
	double distY = std::abs(this->anfang.y - this->ende.y);

	return std::sqrt(std::pow(distX, 2) + std::pow(distY, 2));
}
