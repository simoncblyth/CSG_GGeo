
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

    int repeatIdx = argc > 1 ? atoi(argv[1]) : -1 ;   
    int primIdx = -1 ; 
    int partIdxRel = -1 ; 

    if( repeatIdx != -1 )
    {
        LOG(info) 
            << " CAUTION : partial geometry conversions are for debugging only " 
            << " repeatIdx " << repeatIdx
            ; 
    }

    Opticks ok(argc, argv);
    ok.configure(); 

    bool reverse = SSys::getenvbool("REVERSE", false) ; 
    GGeo* ggeo = GGeo::Load(&ok); 
    //ggeo->dumpParts("CSG_GGeo.main", repeatIdx, primIdx, partIdxRel ) ;

    CSGFoundry foundry ; 
    CSG_GGeo_Convert conv(&foundry, ggeo, reverse ) ; 
    conv.convert(repeatIdx, primIdx, partIdxRel); 

    const char* cfbase = SSys::getenvvar("CFBASE", "/tmp" ); 
    const char* rel = "CSGFoundry" ; 
    // expects existing directory $CFBASE/CSGFoundry 

    foundry.write(cfbase, rel );    

    // read back the foundary and check identical bytes
    CSGFoundry* fd = CSGFoundry::Load(cfbase, rel); 
    int cmp = CSGFoundry::Compare(&foundry, fd );  
    assert( cmp == 0 ); 

    return 0 ; 
}



