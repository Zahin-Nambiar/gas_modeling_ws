#ifndef NRGGASUTILITIES_H
#define NRGGASUTILITIES_H

#include <vector>

namespace nrg_gas
{
std::vector<double> uniform_rn(int count);

double uniform_rn();

double gaussian(double x, double mu, double sigma);
    
} // namespace nrg_gas

#endif //NRGGASUTILITIES_H