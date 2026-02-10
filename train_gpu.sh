#!/bin/bash
#SBATCH --job-name=mmWaveHAR          # 
#SBATCH --gres=gpu:6000ada:1          # 
#SBATCH --time=0-02:00:00             #
#SBATCH --output=output_%j.log        # 
#SBATCH --error=error_%j.log          #


module load miniforge3
source activate


conda activate Py_mmWave_Roformer

python /tmp/pycharm_project_491/mmWaveHAR250905.py