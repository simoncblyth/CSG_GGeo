#!/bin/bash -l

msg="=== $BASH_SOURCE :"
sdir=$(pwd)
name=$(basename $sdir) 

mkdir -p /tmp/$name 

export CUDA_VISIBLE_DEVICES=0


export CFBASE=/tmp/$USER/opticks/CSG_GGeo
mkdir -p ${CFBASE}/CSGFoundry

which $name
$name $*

ls -l ${CFBASE}/CSGFoundry/




