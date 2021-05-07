#!/bin/bash -l

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

