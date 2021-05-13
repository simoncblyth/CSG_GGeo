#include "SSys.hh"
#include "OPTICKS_LOG.hh"
#include "Opticks.hh"
#include "GGeo.hh"

#include "sutil_vec_math.h"
#include "CSGFoundry.h"

#include "CSG_GGeo_Convert.h"

int main(int argc, char** argv)
{
    OPTICKS_LOG(argc, argv);

    Opticks ok(argc, argv);
    ok.configure(); 

    GGeo* ggeo = GGeo::Load(&ok); 

    CSGFoundry foundry ; 
    CSG_GGeo_Convert conv(&foundry, ggeo) ; 
    conv.convert(); 

    bool ops = SSys::getenvbool("ONE_PRIM_SOLID"); 
    if(ops) conv.addOnePrimSolid(); 


    const char* cfbase = SSys::getenvvar("CFBASE", "/tmp" ); 
    const char* rel = "CSGFoundry" ; 

    foundry.write(cfbase, rel );    // expects existing directory $CFBASE/CSGFoundry 

    CSGFoundry* fd = CSGFoundry::Load(cfbase, rel);  // load foundary and check identical bytes
    assert( 0 == CSGFoundry::Compare(&foundry, fd ) );  

    return 0 ; 
}
