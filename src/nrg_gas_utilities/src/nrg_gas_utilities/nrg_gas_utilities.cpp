#include <nrg_gas_utilities/nrg_gas_utilities.h>

#include <random>
#include <algorithm>

using nrg_gas_utilities::GasSource;

namespace nrg_gas
{

std::vector<double> uniform_rn(int count)
{   
    std::random_device rnd_device;
    std::mt19937 mersenne_engine {rnd_device()};  
    std::uniform_real_distribution<double> dist (0, 1.0);
    
    //TODO can I make this return double instead of auto for clarity
    auto gen = [&dist, &mersenne_engine]()  
    {
        return dist(mersenne_engine);
    };

    std::vector<double> rnv(count);
    std::generate(rnv.begin(), rnv.end(), gen);

    return rnv;
}

double uniform_rn()
{   
    std::random_device rnd_device;
    std::mt19937 mersenne_engine {rnd_device()};  // Generates random doubles
    std::uniform_real_distribution<double> dist (0, 1.0);
    
    return dist(mersenne_engine);
}


double gaussian(double x, double mu, double sigma)
{
    return ( 1/(std::sqrt(2*M_PI)*sigma) )*std::exp( -0.5*std::pow((x-mu)/sigma, 2) );
}


visualization_msgs::Marker createSourceMarker(const GasSource& source, const int source_number, const int total_source_number)
{
  visualization_msgs::Marker marker;
  
  marker.header = source.position.header;
  marker.pose.position = source.position.point;
  marker.pose.orientation.w = 1;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;
  marker.color.b = source_number/total_source_number;
  marker.color.g = source_number/total_source_number;
  //marker.color.b = source_number/total_source_number;
  marker.color.a = 1;
  return marker;
}

} // namespace nrg_gas