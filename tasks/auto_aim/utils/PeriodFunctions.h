#pragma once
#include <vector>
#include <cmath>
#include <algorithm>
#include "utils/DataProcessFuncs.h"


std::vector<double> computeModifiedACF(const std::vector<double>& residual);
std::vector<double> lagStackWithDecay(const std::vector<double>& signal, int refineMultiple = 1);