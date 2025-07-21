#!/bin/bash
#SBATCH --job-name=train_1_layer
#SBATCH --output=/home/filippo_mattii/outputs/output_%j.txt
#SBATCH --error=/home/filippo_mattii/errors/error_%j.txt
#SBATCH --partition=gpu
#SBATCH --gres=gpu:1
#SBATCH --cpus-per-task=4
#SBATCH --time=12:00:00
#SBATCH --mem=8G



module load cuda/12.1

source ~/miniconda3/etc/profile.d/conda.sh
conda activate zerostep

python train_zerostep_1_traj_00percent.py