#include <nrg_gas_utilities/nrg_gas.h>
#include <nrg_gas_utilities/GasSource.h>

#include <vector>

namespace nrg_gas
{

using nrg_gas_utilities::GasSource;

struct Particle
{
    GasSource source;
    double weight;
};

class NRGGasSourceLocalization: protected NRGGas
{
public:
    NRGGasSourceLocalization();
    void update( const GasConcentration& gas_measurement, const Vector3Stamped& wind_measurement );
private:
    void initialize(const int& np);
    void reweight( const GasConcentration& gas_measurement, const Vector3Stamped& wind_measurement );
    void resample();
    bool isDegenerate();

    std::vector<Particle> particle_set_;
    std::vector<std::vector<double>> state_space_;
    int np_min_;

    double R_; 
    double Q_;

    ros::NodeHandle nh_;
    
    ros::Publisher visualization_pub_;
};

} // namespace nrg_gas
