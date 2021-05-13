#!/bin/bash -l

usage(){ cat << EOU

::

    ./run.sh --gparts_transform_offset 

    ONE_PRIM_SOLID=1 ./run.sh --gparts_transform_offset 

    GDB=lldb_ ./run.sh -- --gparts_transform_offset


Now that -e option works at CSGOptiX level where the
CF geometry is uploaded to GPU there 
is no point to do that here and bake it into the 
persisted CSGFoundry geometry. As would then need
to manage lots of different directories of CF geometry.

EOU
}


msg="=== $BASH_SOURCE :"
sdir=$(pwd)
name=$(basename $sdir) 

#export ONE_PRIM_SOLID=1 # adds extra debugging solids that reuse existing prim one-by-one

export CFBASE=/tmp/$USER/opticks/CSG_GGeo
outdir=${CFBASE}/CSGFoundry
logdir=${CFBASE}/logs

mkdir -p $outdir 
mkdir -p $logdir 

cd $logdir
which $name
$GDB $name $*

echo $msg outdir:$outdir
ls -l $outdir/

echo $msg logdir:$logdir
ls -l $logdir/

pwd

