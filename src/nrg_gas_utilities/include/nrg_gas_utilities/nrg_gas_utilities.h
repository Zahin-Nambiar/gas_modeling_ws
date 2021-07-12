#ifndef NRGGASUTILITIES_H
#define NRGGASUTILITIES_H

#include <nrg_gas_utilities/GasSource.h>

#include <visualization_msgs/Marker.h>

#include <vector>

namespace nrg_gas
{
std::vector<double> uniform_rn(int count);

double uniform_rn();

double gaussian(double x, double mu, double sigma);

visualization_msgs::Marker createSourceMarker(const nrg_gas_utilities::GasSource& source,const int source_number,const int total_source_number);

} // namespace nrg_gas

#endif //NRGGASUTILITIES_H