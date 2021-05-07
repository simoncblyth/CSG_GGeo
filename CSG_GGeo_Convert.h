#pragma once

struct CSGFoundry ;
struct CSGSolid ; 
struct CSGPrim ; 
struct CSGNode ; 

class GGeo ;
class GParts ; 

struct CSG_GGeo_Convert
{   
    CSGFoundry* foundry ; 
    const GGeo* ggeo ; 
    bool reverse ; 
    float splay ; 

    static const char* Label(unsigned repeatIdx ); 

    CSG_GGeo_Convert(CSGFoundry* foundry, const GGeo* ggeo ) ; 

    void convert(int repeatIdx,  int primIdx, int partIdxRel );
    void convert_();

    CSGSolid* convert_(unsigned repeatIdx );
    void addInstances(unsigned repeatIdx );

    CSGPrim*  convert_(const GParts* comp, unsigned primIdx );
    CSGNode*  convert_(const GParts* comp, unsigned primIdx, unsigned partIdxRel );
};


