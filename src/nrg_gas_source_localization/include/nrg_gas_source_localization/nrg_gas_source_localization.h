#include <nrg_gas_utilities/nrg_gas.h>

#include <vector>

namespace nrg_gas
{

struct Particle
{
    
};

class NRGGasSourceLocalization: protected NRGGas
{
public:

private:
    std::vector<Particle> particle_set_;
};

} // namespace nrg_gas
