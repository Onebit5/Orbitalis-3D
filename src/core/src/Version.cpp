#include <orbitalis/core/Version.hpp>

namespace orbitalis {

const char* version_banner()
{
    return "Orbitalis-3D " ORBITALIS_VERSION_STRING;
}

const char* milestone_name()
{
    return "0.0.4 - newtonian gravity, brute force";
}

}  // namespace orbitalis
