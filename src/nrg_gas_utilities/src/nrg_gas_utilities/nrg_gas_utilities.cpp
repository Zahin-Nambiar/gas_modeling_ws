#include <nrg_gas_utilities/nrg_gas_utilities.h>

#include <random>
#include <algorithm>

namespace nrg_gas
{

std::vector<double> uniform_rn(int count){   
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

double uniform_rn(){   
    std::random_device rnd_device;
    std::mt19937 mersenne_engine {rnd_device()};  // Generates random doubles
    std::uniform_real_distribution<double> dist (0, 1.0);
    
    return dist(mersenne_engine);
}


double gaussian(double x, double mu, double sigma){
    return ( 1/(std::sqrt(2*M_PI)*sigma) )*std::exp( -0.5*std::pow((x-mu)/sigma, 2) );
}

} // namespace nrg_gas