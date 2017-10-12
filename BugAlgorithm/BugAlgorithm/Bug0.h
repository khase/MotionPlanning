#pragma once
#include "BugAlgorithm.h"
class Bug0 :
	public BugAlgorithm
{
public:
	Bug0(const std::string& name);
	~Bug0();

	bool update(Box obstacle[], Box robot[], int nObst);
	int obstacleInWay(Box obstacle[], Box robot[], int nObst);
};

