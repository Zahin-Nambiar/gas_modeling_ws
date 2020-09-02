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
    void update(GasConcentration& gas_measurement, Vector3 wind_measurement);
private:
    void initialize(int np);
    void reweight();
    void resample();
    void isNeffLow();

    std::vector<Particle> particle_set_;
    std::vector<std::vector<double>> state_space_;
    int np_min_;
};

} // namespace nrg_gas
