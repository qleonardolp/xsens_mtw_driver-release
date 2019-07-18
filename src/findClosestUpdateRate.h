#ifndef FINDCLOSESTUPDATERATE_H
#define FINDCLOSESTUPDATERATE_H

#include <xsens/xsintarray.h>

#include <sstream>

int findClosestUpdateRate(const XsIntArray& supportedUpdateRates, const int desiredUpdateRate);

#endif