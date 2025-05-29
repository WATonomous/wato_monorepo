#!/bin/bash

# Download desired dataset .tgz file from https://www.nuscenes.org/nuscenes#download

filepath=$1
filename="$(basename ${filepath})"
dataname="${filename%.*}"
echo "${dataname}" > ../../../nusc_info/dataset_name.txt

datadir="${dataname}_data"
mkdir -p "${datadir}"
cd "${datadir}"
tar -xvzf "${filepath}"

