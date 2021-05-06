#!/bin/bash -l

msg="=== $BASH_SOURCE :"
sdir=$(pwd)
name=$(basename $sdir) 

export CFBASE=/tmp/$USER/opticks/CSG_GGeo
outdir=${CFBASE}/CSGFoundry

mkdir -p $outdir

which $name
$name $*

ls -l $outdir/


