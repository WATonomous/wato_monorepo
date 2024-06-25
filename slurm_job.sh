#!/bin/bash
#SBATCH --job-name=my_job
#SBATCH --cpus-per-task=1
#SBATCH --mem=1G
#SBATCH --gres tmpdisk:1024
#SBATCH --time=00:10:00
#SBATCH --output=logs/%j-%x.out # %j: job ID, %x: job name. Reference: <https://slurm.schedmd.com/sbatch.html#lbAH>
 
slurm-start-dockerd.sh
export DOCKER_HOST=unix:///tmp/run/docker.sock
docker run --rm hello-world