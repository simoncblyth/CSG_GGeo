#!/bin/bash -l

usage(){ cat << EOU

::

    ./run.sh
    ./run.sh -e ~8,      # skip conversion of mm 8      FAILS to render because of getGAS lookup fail
    ./run.sh -e ~8,9     # skip conversion of mm 8,9    renders OK because no gaps 

EOU
}


msg="=== $BASH_SOURCE :"
sdir=$(pwd)
name=$(basename $sdir) 

export CFBASE=/tmp/$USER/opticks/CSG_GGeo
outdir=${CFBASE}/CSGFoundry
logdir=${CFBASE}/logs

mkdir -p $outdir 
mkdir -p $logdir 

cd $logdir
which $name
$name $*

echo $msg outdir:$outdir
ls -l $outdir/

echo $msg logdir:$logdir
ls -l $logdir/

pwd

