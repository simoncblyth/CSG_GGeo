#pragma once

#include "plog/Severity.h"

struct CSGFoundry ;
struct CSGSolid ; 
struct CSGPrim ; 
struct CSGNode ; 

class GGeo ;
class GParts ; 

struct CSG_GGeo_Convert
{   
    static const plog::Severity LEVEL ; 

    CSGFoundry* foundry ; 
    const GGeo* ggeo ; 
    const Opticks* ok ; 

    bool reverse ; 
    float splay ; 

    static const char* Label(unsigned repeatIdx ); 

    CSG_GGeo_Convert(CSGFoundry* foundry, const GGeo* ggeo ) ; 
    void init();

    void convert(int repeatIdx=-1,  int primIdx=-1, int partIdxRel=-1 );
    void convert_();

    CSGSolid* convert_(unsigned repeatIdx );
    void addInstances(unsigned repeatIdx );

    CSGPrim*  convert_(const GParts* comp, unsigned primIdx );
    CSGNode*  convert_(const GParts* comp, unsigned primIdx, unsigned partIdxRel );
};


