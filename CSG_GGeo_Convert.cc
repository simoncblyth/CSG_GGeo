#include <iostream>
#include <iomanip>
#include <vector>

#include "SSys.hh"
#include "NGLMExt.hpp"
#include "GLMFormat.hpp"

#include "Opticks.hh"

#include "GGeo.hh"
#include "GGeoLib.hh"
#include "GParts.hh"
#include "GMergedMesh.hh"

#include "sutil_vec_math.h"
#include "CSGFoundry.h"
#include "CSGSolid.h" 
#include "CSGPrim.h" 
#include "CSGNode.h" 
#include "qat4.h"
#include "AABB.h"

#include "PLOG.hh"
#include "CSG_GGeo_Convert.h"


const plog::Severity CSG_GGeo_Convert::LEVEL = PLOG::EnvLevel("CSG_GGeo_Convert", "DEBUG"); 


CSG_GGeo_Convert::CSG_GGeo_Convert(CSGFoundry* foundry_, const GGeo* ggeo_ ) 
    : 
    foundry(foundry_),
    ggeo(ggeo_),
    ok(ggeo->getOpticks()),
    reverse(SSys::getenvbool("REVERSE")),
    splay(SSys::getenvfloat("SPLAY", 0.f ))
{
    LOG(info) 
        << " reverse " << reverse
        << " splay " << splay
        ;  

    init(); 
}


void CSG_GGeo_Convert::init()
{
    ggeo->getMeshNames(foundry->name); 
    LOG(info) 
       << std::endl
       << " foundry.name.size " << foundry->name.size() 
       ;
}



void CSG_GGeo_Convert::convert(int repeatIdx,  int primIdx, int partIdxRel )
{

    if( repeatIdx > -1 || primIdx > -1 || partIdxRel > -1 )
    {
        LOG(info) 
            << " CAUTION : partial geometry conversions are for debugging only " 
            << " repeatIdx " << repeatIdx
            << " primIdx " << primIdx
            << " partIdxRel " << partIdxRel
            ; 
    }

    if( repeatIdx > -1 && primIdx > -1 && partIdxRel > -1 )  
    {   
        LOG(info) << "fully defined : convert just a single node " ; 
        const GParts* comp = ggeo->getCompositeParts(repeatIdx) ; 
        convert_(comp, primIdx, partIdxRel);
    }
    else if( repeatIdx > -1 && primIdx > -1  )   
    {   
        LOG(info) << "convert all nodes in a single Prim " ; 
        const GParts* comp = ggeo->getCompositeParts(repeatIdx) ; 
        convert_(comp, primIdx);
    }
    else if( repeatIdx > -1 )  
    {   
        LOG(info) << " convert all Prim in a single repeat composite Solid " ; 
        convert_(repeatIdx);
    }
    else                
    { 
        LOG(info) << "convert all solids (default)" ; 
        convert_();
    }
} 

void CSG_GGeo_Convert::convert_()
{
    unsigned numRepeat = ggeo->getNumMergedMesh(); 
    for(unsigned repeatIdx=0 ; repeatIdx < numRepeat ; repeatIdx++)
    {
        if(ok->isEnabledMergedMesh(repeatIdx))
        {
            LOG(error) << "proceeding with convert for repeatIdx " << repeatIdx ;  
            convert_(reverse ? numRepeat - 1 - repeatIdx : repeatIdx ); 
        }
        else
        {
            LOG(error) << "skipping convert for repeatIdx " << repeatIdx ;  
        }
    }
}

void CSG_GGeo_Convert::addInstances(unsigned repeatIdx )
{
    unsigned nmm = ggeo->getNumMergedMesh(); 
    assert( repeatIdx < nmm ); 
    const GMergedMesh* mm = ggeo->getMergedMesh(repeatIdx); 
    unsigned num_inst = mm->getNumITransforms() ;

    //LOG(LEVEL) << " nmm " << nmm << " repeatIdx " << repeatIdx << " num_inst " << num_inst ; 

    for(unsigned i=0 ; i < num_inst ; i++)
    {
        glm::mat4 it = mm->getITransform_(i); 
        qat4 instance(glm::value_ptr(it)) ;   
        unsigned ins_idx = foundry->inst.size() ;
        unsigned gas_idx = repeatIdx ; 
        unsigned ias_idx = 0 ; 
        instance.setIdentity( ins_idx, gas_idx, ias_idx );
        foundry->inst.push_back( instance );
    }
}


const char* CSG_GGeo_Convert::Label(unsigned repeatIdx ) // static
{
    std::stringstream ss ; 
    ss << "r" << std::setfill('0') << std::setw(3) << repeatIdx ; 
    std::string s = ss.str();  
    return strdup(s.c_str()) ; 
}


CSGSolid* CSG_GGeo_Convert::convert_( unsigned repeatIdx )
{
    unsigned nmm = ggeo->getNumMergedMesh(); 
    assert( repeatIdx < nmm ); 
    const GMergedMesh* mm = ggeo->getMergedMesh(repeatIdx); 
    unsigned num_inst = mm->getNumITransforms() ;

    const GParts* comp = ggeo->getCompositeParts(repeatIdx) ; 
    unsigned numPrim = comp->getNumPrim();
    const char* label = CSG_GGeo_Convert::Label(repeatIdx) ; 

    LOG(info)
        << " repeatIdx " << repeatIdx 
        << " nmm " << nmm
        << " numPrim " << numPrim
        << " label " << label 
        << " num_inst " << num_inst 
        ;   

    CSGSolid* so = foundry->addSolid(numPrim, label );  // primOffset captured into CSGSolid 
    assert(so); 

    AABB bb = {} ;

    for(unsigned primIdx=0 ; primIdx < numPrim ; primIdx++)
    {   
        CSGPrim* prim = convert_(comp, primIdx); 
        bb.include_aabb( prim->AABB() );
    
        unsigned globalPrimIdx = so->primOffset + primIdx ; 
        prim->setSbtIndexOffset(globalPrimIdx) ;
        prim->setRepeatIdx(repeatIdx); 
        prim->setPrimIdx(primIdx); 

        //LOG(info) << prim->desc() ;
    }   
    so->center_extent = bb.center_extent() ;  

    addInstances(repeatIdx); 

    LOG(info) << " solid.bb " <<  bb ;
    LOG(info) << " solid.desc " << so->desc() ;

    return so ; 
}


CSGPrim* CSG_GGeo_Convert::convert_(const GParts* comp, unsigned primIdx )
{
    unsigned numPrim = comp->getNumPrim();
    assert( primIdx < numPrim ); 
    unsigned numParts = comp->getNumParts(primIdx) ;
    unsigned meshIdx = comp->getMeshIndex(primIdx);    // aka lvIdx

    // on adding a prim the node/tran/plan offsets are captured into the Prim 
    // from the sizes of the foundry vectors

    CSGPrim* prim = foundry->addPrim(numParts, meshIdx );   
    assert(prim) ; 

    AABB bb = {} ;
    for(unsigned partIdxRel=0 ; partIdxRel < numParts ; partIdxRel++ )
    {
        CSGNode* n = convert_(comp, primIdx, partIdxRel); 
        bb.include_aabb( n->AABB() );   
    }
    prim->setAABB( bb.data() ); 
    return prim ; 
}


/**
CSG_GGeo_Convert::convert_
----------------------------

primIdx
    0:numPrim-1 identifying the Prim (aka layer) within the composite 

partIdxRel 
    relative part(aka node) index 0:numPart-1 within the Prim, used together with 
    the partOffset of the Prim to yield the absolute partIdx within the composite

**/




CSGNode* CSG_GGeo_Convert::convert_(const GParts* comp, unsigned primIdx, unsigned partIdxRel )
{
    //unsigned repeatIdx = comp->getRepeatIndex();  // set in GGeo::deferredCreateGParts
    unsigned partOffset = comp->getPartOffset(primIdx) ;
    unsigned partIdx = partOffset + partIdxRel ;
    unsigned idx = comp->getIndex(partIdx);
    assert( idx == partIdx ); 

    std::string tag = comp->getTag(partIdx); 
    unsigned tc = comp->getTypeCode(partIdx);

    unsigned gtran = 0 ; 
    const Tran<float>* tv = nullptr ; 

    if( splay != 0.f )    // splaying currently prevents the real transform from being used 
    {
        tv = Tran<float>::make_translate(0.f, float(primIdx)*splay, float(partIdxRel)*splay ); 
    }
    else
    {
        gtran = comp->getGTransform(partIdx);
        if( gtran > 0 )
        {
            glm::mat4 t = comp->getTran(gtran-1,0) ;
            glm::mat4 v = comp->getTran(gtran-1,1); 
            tv = new Tran<float>(t, v); 
        }
    }

    // Note that repeatedly getting the same gtran for different part(aka node) 
    // will repeatedly add that same transform to the foundry even when its the identity transform.
    // So this is profligate in duplicated transforms.

    unsigned tranIdx = tv ?  1 + foundry->addTran(*tv) : 0 ; 

    const float* param = comp->getPartValues(partIdx, 0, 0 );  

    /*
    LOG(info) 
        << CSGNode::Addr(repeatIdx, primIdx, partIdxRel )
        << " partIdx " << std::setw(3) << partIdx
        << " GTransform " << std::setw(3) << gtran
    //    << " param: " << CSGNode::Desc( param, 6 ) 
        << " tranIdx " << std::setw(4) << tranIdx 
        << " tv " <<  ( tv ? tv->brief() : "-" )  
        ; 

    */

    const float* aabb = nullptr ;  
    CSGNode* n = foundry->addNode(CSGNode::Make(tc, param, aabb ));
    n->setIndex(partIdx); 
    n->setAABBLocal(); 
    n->setTransform(tranIdx); 

    if(tranIdx > 0 )
    {    
        const qat4* q = foundry->getTran(tranIdx-1u) ;

        q->transform_aabb_inplace( n->AABB() );
    }

    return n ; 

}


