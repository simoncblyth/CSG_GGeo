#include <iostream>
#include <iomanip>
#include <vector>

#include "SStr.hh"
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
    reverse(SSys::getenvbool("REVERSE"))
{
    LOG(info) 
        << " reverse " << reverse
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
        convertNode(comp, primIdx, partIdxRel);
    }
    else if( repeatIdx > -1 && primIdx > -1  )   
    {   
        LOG(info) << "convert all nodes in a single Prim " ; 
        const GParts* comp = ggeo->getCompositeParts(repeatIdx) ; 
        convertPrim(comp, primIdx);
    }
    else if( repeatIdx > -1 )  
    {   
        LOG(info) << " convert all Prim in a single repeat composite Solid " ; 
        convertSolid(repeatIdx);
    }
    else                
    { 
        LOG(info) << "convert all solids (default)" ; 
        convertAllSolid();
    }
} 

void CSG_GGeo_Convert::convertAllSolid()
{
    unsigned numRepeat = ggeo->getNumMergedMesh(); 
    for(unsigned repeatIdx=0 ; repeatIdx < numRepeat ; repeatIdx++)
    {
        if(ok->isEnabledMergedMesh(repeatIdx))
        {
            LOG(error) << "proceeding with convert for repeatIdx " << repeatIdx ;  
            convertSolid(reverse ? numRepeat - 1 - repeatIdx : repeatIdx ); 
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



CSGSolid* CSG_GGeo_Convert::convertSolid( unsigned repeatIdx )
{
    unsigned nmm = ggeo->getNumMergedMesh(); 
    assert( repeatIdx < nmm ); 
    const GMergedMesh* mm = ggeo->getMergedMesh(repeatIdx); 
    unsigned num_inst = mm->getNumITransforms() ;

    const GParts* comp = ggeo->getCompositeParts(repeatIdx) ; 
    unsigned numPrim = comp->getNumPrim();
    std::string rlabel = CSGSolid::MakeLabel('r',repeatIdx) ; 

    LOG(info)
        << "CSG_GGeo_Convert::convertSolid"
        << " repeatIdx " << repeatIdx 
        << " nmm " << nmm
        << " numPrim " << numPrim
        << " rlabel " << rlabel 
        << " num_inst " << num_inst 
        ;   

    CSGSolid* so = foundry->addSolid(numPrim, rlabel.c_str() );  // primOffset captured into CSGSolid 
    assert(so); 

    AABB bb = {} ;

    for(unsigned primIdx=0 ; primIdx < numPrim ; primIdx++)
    {   
        unsigned globalPrimIdx = so->primOffset + primIdx ;
        unsigned globalPrimIdx_0 = foundry->getNumPrim() ; 
        assert( globalPrimIdx == globalPrimIdx_0 ); 

        CSGPrim* prim = convertPrim(comp, primIdx); 
        bb.include_aabb( prim->AABB() );

        unsigned sbtIdx = prim->sbtIndexOffset() ; 
        //assert( sbtIdx == globalPrimIdx  );  
        assert( sbtIdx == primIdx  );  

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


/**
CSG_GGeo_Convert::convertPrim
--------------------------------

on adding a prim the node/tran/plan offsets are captured into the Prim 
from the sizes of the foundry vectors

TODO: suspect tran and plan offsets are not used, and should be removed
as are always using absolute tran and plan addressing 

**/

CSGPrim* CSG_GGeo_Convert::convertPrim(const GParts* comp, unsigned primIdx )
{
    unsigned numPrim = comp->getNumPrim();
    assert( primIdx < numPrim ); 
    unsigned numParts = comp->getNumParts(primIdx) ;
    unsigned meshIdx = comp->getMeshIndex(primIdx);    // aka lvIdx

    assert( foundry->last_added_solid ); 
    bool is_r8 = foundry->last_added_solid->labelMatch("r8"); 

    int nodeOffset_ = -1 ; 
    CSGPrim* prim = foundry->addPrim(numParts, nodeOffset_ );   
    prim->setMeshIdx(meshIdx);   
    assert(prim) ; 

    AABB bb = {} ;

    //bool kludge_skip_aabb = false ; 
   
    for(unsigned partIdxRel=0 ; partIdxRel < numParts ; partIdxRel++ )
    {
        CSGNode* n = convertNode(comp, primIdx, partIdxRel); 
        float* naabb = n->AABB();  

        if(is_r8)
        {
            std::cout 
                << "CSG_GGeo_Convert::convertPrim is_r8" 
                << " primIdx " << std::setw(3) << primIdx 
                << " partIdxRel " << std::setw(3) << partIdxRel
                << AABB::Desc(naabb)
                << std::endl 
                ;   
        } 

        bb.include_aabb( naabb );  


/*
        kludge_skip_aabb = is_r8 && *(naabb+0) < -17810.00 ; 

        if(!kludge_skip_aabb)
        { 
            bb.include_aabb( naabb );  
        }
        else
        {
            std::cout 
                << "CSG_GGeo_Convert::convertPrim kludge_skip_aabb " 
                << std::endl
                ; 
        }
*/


    }


    const float* bb_data = bb.data(); 

    if(is_r8)
    {
        std::cout << "is_r8.Prim.AABB " << AABB::Desc(bb_data) << std::endl ; 
    }       

    prim->setAABB( bb_data ); 

    return prim ; 
}

/**
CSG_GGeo_Convert::convertNode
-------------------------------

primIdx
    0:numPrim-1 identifying the Prim (aka layer) within the composite 

partIdxRel 
    relative part(aka node) index 0:numPart-1 within the Prim, used together with 
    the partOffset of the Prim to yield the absolute partIdx within the composite


Note that repeatedly getting the same gtran for different part(aka node) 
will repeatedly add that same transform to the foundry even when its the identity transform.
So this is profligate in duplicated transforms.

Observing many complemented nodes that appear should not be complemented, even in some 
single node Prim such as "r1/4" and 31 node "r8/0".

**/


CSGNode* CSG_GGeo_Convert::convertNode(const GParts* comp, unsigned primIdx, unsigned partIdxRel )
{
    unsigned repeatIdx = comp->getRepeatIndex();  // set in GGeo::deferredCreateGParts
    unsigned partOffset = comp->getPartOffset(primIdx) ;
    unsigned partIdx = partOffset + partIdxRel ;
    unsigned idx = comp->getIndex(partIdx);
    assert( idx == partIdx ); 

    std::string tag = comp->getTag(partIdx); 
    unsigned tc = comp->getTypeCode(partIdx);

    const Tran<float>* tv = nullptr ; 
    unsigned gtran = comp->getGTransform(partIdx);
    if( gtran > 0 )
    {
        glm::mat4 t = comp->getTran(gtran-1,0) ;
        glm::mat4 v = comp->getTran(gtran-1,1); 
        tv = new Tran<float>(t, v); 
    }

    unsigned tranIdx = tv ?  1 + foundry->addTran(*tv) : 0 ; 

    const float* param = comp->getPartValues(partIdx, 0, 0 );  
    bool complement = comp->getComplement(partIdx);

    const float* aabb = nullptr ;  
    CSGNode* n = foundry->addNode(CSGNode::Make(tc, param, aabb ));
    n->setIndex(partIdx); 
    n->setAABBLocal(); 
    n->setTransform(tranIdx); 
    n->setComplement(complement); 

    if(complement) 
        std::cout 
            << "CSG_GGeo_Convert::convertNode"
            << " repeatIdx " << repeatIdx 
            << " primIdx " << primIdx 
            << " partIdxRel " << partIdxRel 
            << " complement " << complement
            << std::endl 
            ; 

    assert( n->complement() == complement ); 

    if(tranIdx > 0 )
    {    
        const qat4* q = foundry->getTran(tranIdx-1u) ;
        q->transform_aabb_inplace( n->AABB() );
    }

    return n ; 

}



/**
CSG_GGeo_Convert::addOnePrimSolid
-------------------------------------

CSGSolid typically have multiple Prim, in case of the remainder solid (solidIdx 0) 
there can even be thousands of Prim in a single "Solid".

For debugging purposes it is useful to add extra solids that each have only a single 
CSGPrim allowing rendering of each "layer" of a Solid without adding separate 
code paths to do this, just reusing the existing machinery applied to the extra  solids.

Note that repeatIdx/solidIdx/gasIdx are mostly referring to the same thing 
with nuances of which stage they are at. 
**/



void CSG_GGeo_Convert::addOnePrimSolid(unsigned solidIdx)
{
    const CSGSolid* orig = foundry->getSolid(solidIdx);    

    if(orig->numPrim == 1 ) 
    {
        LOG(info) << "already single prim solid, no point adding another" ; 
        return ; 
    }  

    for(unsigned primIdx=orig->primOffset ; primIdx < unsigned(orig->primOffset+orig->numPrim) ; primIdx++)  
    {
        unsigned numPrim = 1 ; 
        int primOffset_ = primIdx ;   // note absolute primIdx

        unsigned primIdxRel = primIdx - orig->primOffset ; 

        std::string rp_label = CSGSolid::MakeLabel('r', solidIdx, 'p', primIdxRel ) ;   

        // NB not adding Prim just reusing pre-existing in separate Solid
        CSGSolid* pso = foundry->addSolid(numPrim, rp_label.c_str(), primOffset_ ); 

        AABB bb = {} ;
        const CSGPrim* prim = foundry->getPrim(primIdx) ; //  note absolute primIdx
        bb.include_aabb( prim->AABB() );

        pso->center_extent = bb.center_extent() ;  

        pso->type = ONE_PRIM_SOLID ; 
        LOG(info) << pso->desc() ;  
    }   
}




void CSG_GGeo_Convert::addOnePrimSolid()
{
    unsigned num_solid_standard = foundry->getNumSolid(STANDARD_SOLID) ; 

    std::vector<unsigned> one_prim_solid ; 
    SStr::GetEVector(one_prim_solid, "ONE_PRIM_SOLID", "1,2,3,4,5,6,7,8" ); 

    LOG(error) << "foundry.desc before " << foundry->desc(); 
    for(unsigned i=0 ; i < one_prim_solid.size() ; i++)
    {
        unsigned solidIdx = one_prim_solid[i] ; 
        if( solidIdx < num_solid_standard )
        {
            addOnePrimSolid(solidIdx);       
        }
        else
        {
            LOG(error) << " requested ONE_PRIM_SOLID solidIdx out of range " << solidIdx ; 
        }
    }
    LOG(error) << "foundry.desc after " << foundry->desc(); 
}



/**
CSG_GGeo_Convert::addOneNodeSolid
------------------------------------

Unlike OnePrimSolid this needs to create 
both the solid and the Prim which references a pre-existing Node.

This makes sense for leaf nodes, but for operator nodes
a single node solid makes no sense : would need to construct subtrees in that case 
and also create new node sequences. This is because cannot partially reuse preexisting 
node complete binary trees to form subtrees because the node order 
would be wrong for subtrees.

**/

void CSG_GGeo_Convert::addOneNodeSolid()
{
    unsigned num_solid_standard = foundry->getNumSolid(STANDARD_SOLID) ; 

    std::vector<unsigned> one_node_solid ; 
    SStr::GetEVector(one_node_solid, "ONE_NODE_SOLID", "1,2,3,4,5,6,7,8" ); 

    LOG(error) << "foundry.desc before " << foundry->desc(); 

    for(unsigned i=0 ; i < one_node_solid.size() ; i++)
    {
        unsigned solidIdx = one_node_solid[i] ; 
        if( solidIdx < num_solid_standard )
        {
            addOneNodeSolid(solidIdx);       
        }
        else
        {
            LOG(error) << " requested ONE_NODE_SOLID solidIdx out of range " << solidIdx ; 
        }
    }
    LOG(error) << "foundry.desc after " << foundry->desc(); 

}


/**
CSG_GGeo_Convert::addOneNodeSolid
-------------------------------------

Invokes the below addOneNodeSolid for each prim of the original solidIdx 

**/

void CSG_GGeo_Convert::addOneNodeSolid(unsigned solidIdx)
{
    LOG(info) << " solidIdx " << solidIdx ; 
    const CSGSolid* orig = foundry->getSolid(solidIdx);    

    for(unsigned primIdx=orig->primOffset ; primIdx < unsigned(orig->primOffset+orig->numPrim) ; primIdx++)  
    {
        unsigned primIdxRel = primIdx - orig->primOffset ; 

        addOneNodeSolid(solidIdx, primIdx, primIdxRel);   
    }   
}


/**
CSG_GGeo_Convert::addOneNodeSolid
-----------------------------------

Adds solids for each non-operator leaf node of the Solid/Prim 
identified by solidIdx/primIdx/primIdxRel with label of form "R1P0N0" 
that shows the origin of this debugging solid. 

**/

void CSG_GGeo_Convert::addOneNodeSolid(unsigned solidIdx, unsigned primIdx, unsigned primIdxRel)
{
    const CSGPrim* prim = foundry->getPrim(primIdx);      

    for(unsigned nodeIdxRel=0 ; nodeIdxRel < unsigned(prim->numNode()) ; nodeIdxRel++ )
    {
        unsigned nodeIdx = prim->nodeOffset() + nodeIdxRel ; 

        const CSGNode* node = foundry->getNode(nodeIdx); 

        if(node->is_operator() || node->is_zero()) continue ; 

        AABB bb = {} ;
        bb.include_aabb( node->AABB() );   

        std::string rpn_label = CSGSolid::MakeLabel('R', solidIdx, 'P', primIdxRel, 'N', nodeIdxRel ) ;   

        unsigned numPrim = 1 ;  
        CSGSolid* rpn_solid = foundry->addSolid(numPrim, rpn_label.c_str()  ); 
        rpn_solid->center_extent = bb.center_extent() ;  

        int numNode = 1 ; 
        int nodeOffset = nodeIdx ;   // re-using the node 

        CSGPrim* rpn_prim = foundry->addPrim(numNode, nodeOffset ) ; 
        rpn_prim->setMeshIdx(prim->meshIdx()); 
        rpn_prim->setAABB( bb.data() ); 
        
        //rpn_prim->setMeshIdx(prim->meshIdx()); 
        rpn_prim->setRepeatIdx(prim->repeatIdx()); 
        rpn_prim->setPrimIdx(prim->primIdx()); 

        rpn_solid->type = ONE_NODE_SOLID ; 
        rpn_solid->center_extent = bb.center_extent() ;  

        LOG(info) << rpn_solid->desc() ;  
    }
}



void CSG_GGeo_Convert::addDeepCopySolid()
{
    unsigned num_solid_standard = foundry->getNumSolid(STANDARD_SOLID) ; 

    std::vector<unsigned> deep_copy_solid ; 
    SStr::GetEVector(deep_copy_solid, "DEEP_COPY_SOLID", "1,2,3,4" ); 

    LOG(error) << "foundry.desc before " << foundry->desc(); 

    for(unsigned i=0 ; i < deep_copy_solid.size() ; i++)
    {
        unsigned solidIdx = deep_copy_solid[i] ; 
        if( solidIdx < num_solid_standard )
        {
            foundry->addDeepCopySolid(solidIdx);       
        }
        else
        {
            LOG(error) << " requested DEEP_COPY_SOLID solidIdx out of range " << solidIdx ; 
        }
    }
    LOG(error) << "foundry.desc after " << foundry->desc(); 
}


void CSG_GGeo_Convert::kludgeScalePrimBBox()
{
    const char* kludge_scale_prim_bbox = SSys::getenvvar("KLUDGE_SCALE_PRIM_BBOX", nullptr); 
    assert(kludge_scale_prim_bbox);  
    
    LOG(error) << "foundry.desc before " << foundry->desc(); 

    float dscale = 0.1f ; 
    foundry->kludgeScalePrimBBox(kludge_scale_prim_bbox, dscale );

    LOG(error) << "foundry.desc after " << foundry->desc(); 
}




