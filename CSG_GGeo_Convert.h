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


    CSG_GGeo_Convert(CSGFoundry* foundry, const GGeo* ggeo ) ; 
    void init();

    void convert();   

    // collect inputs for creating GPU boundary and scintillation textures 
    void convertBndLib() ;
    void convertScintillatorLib() ;
    void convertGeometry(int repeatIdx=-1,  int primIdx=-1, int partIdxRel=-1 );


    // below all for geometry conversion 

    void convertAllSolid();
    CSGSolid* convertSolid(unsigned repeatIdx );
    void addInstances(unsigned repeatIdx );

    CSGPrim*  convertPrim(const GParts* comp, unsigned primIdx );
    CSGNode*  convertNode(const GParts* comp, unsigned primIdx, unsigned partIdxRel );


    // below is called non-standardly during debugging when envvar ONE_PRIM_SOLID is defined 
    void addOnePrimSolid();
    void addOnePrimSolid(unsigned solidIdx);

    // below is called non-standardly during debugging when envvar ONE_NODE_SOLID is defined 
    void addOneNodeSolid();
    void addOneNodeSolid(unsigned solidIdx);
    void addOneNodeSolid(unsigned solidIdx, unsigned primIdx, unsigned primIdxRel);

    // below is called non-standardly during debugging when envvar DEEP_COPY_SOLID is defined 
    void addDeepCopySolid();

    // below is called non-standardly during debugging when envvar KLUDGE_SCALE_PRIM_BBOX is defined 
    void kludgeScalePrimBBox();

};


