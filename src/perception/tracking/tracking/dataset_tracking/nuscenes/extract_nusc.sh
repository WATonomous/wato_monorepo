#!/bin/bash

filepath=$1
filename="$(basename ${filepath})"
dataname="${filename%.*}"
echo "${dataname}" > dataset_name.txt

datadir="${dataname}_data"
mkdir -p "${datadir}"
cd "${datadir}"
tar -xvzf "${filepath}"

