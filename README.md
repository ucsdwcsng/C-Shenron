# C-Shenron: A Realistic Radar Simulator for End-to-End Autonomous Driving in CARLA
This repository contains the official implementation used for the paper "A Realistic Radar Simulator for End-to-End Autonomous Driving in CARLA". The simulator is designed to generate realistic radar data for autonomous driving tasks, enhancing the capabilities of the CARLA simulator.

## Overview
C-Shenron is a high-fidelity radar simulation framework integrated with the CARLA simulator, enabling realistic, physics-based radar data generation using LiDAR and camera inputs. It supports customizable radar sensor setups and generates data suitable for End-to-End (E2E) autonomous driving pipelines, especially for transformer-based models like Transfuser++.

Key features:
- Realistic radar data using physics-based Shenron model
- Supports multiple radar views (Front, Back, Left, Right)
- End-to-end integration with imitation learning pipelines
- Easily scalable data collection and evaluation setup
- Demonstrated performance gains over LiDAR-camera baselines (+3% Driving Score)

## Data Collection
First we need to generate bash scripts for both starting carla simulator and data collection. `data_generation_bash_scripts.py` will generate the scripts into `Data_Collection_Scripts` directory which has two sub-folders:
1. `Start_Carla_Job_Scripts` contains scripts to start carla simulator and run the data collection scripts
2. `Job_Files` contains the data collection scripts

#### Generating scripts:
```shell
python3 data_generation_bash_scripts.py
```

#### To start data collection
```shell
bash Data_Collection_Scripts/Start_Carla_Job_Scripts/job0.sh
```
This is an example, you can run any of the files from `bash Data_Collection_Scripts/Start_Carla_Job_Scripts`.

Refer to [this repository](https://github.com/ucsdwcsng/nautilus-kubernetes-howto/blob/main/parallelization.md) on how to run data collection scrips parallely by running each script in a pod.

> **Note:** The generated data will be updated shortly as we are currently figuring out the best storage solution for the data. We will update the README with the link to the data once it is available.

## Training the model
#### Training:
```shell
bash team_code/shell_train.sh
```

Arguments for `team_code/train.py`:
1. `id` - Specifies the sub-directory where the trained model will be stored
2. `continue_epoch` - Use only when you want to use pre-trained model
    - `0` to train from epoch 0
    - `1` to train from epoch where pre-trained model left it
3. `radar_channels` - Select radar from carla or simulation
    - `2` to use carla's front and back radar
    - `<anything else>` to use radar data from SHENRON
4. `radar_cat` - Select the radar concatenation model from SHENRON
    - `1` to use front and back concatenation
    - `2` to use front, back, left and right concatenation
5. `use_radar` - To use radar data for training
6. `use_lidar` - To use lidar data for training

## Evaluation
Similar to data collection, we need to generate bash scripts for both starting carla simulator and data collection.
1. `Evaluation_Scripts/generate_run_bashs.py` generates the bash scripts to start carla simulator and running the evaluation scripts into `Start_Carla_Job_Scripts`
2. `Evaluation_Scripts/evaluation_bash_scripts.py` generates the evaluation scripts into `Job_Files`

You can vary all the evaluation parameters in `evaluation_bash_scripts.py`.

#### Running Evaluations:
```shell
bash /Evaluation_Scripts/Start_Carla_Job_Scripts/job0.sh
```

Again, this is an example and you can run any of the files from the `Start_Carla_Job_Scripts` and parallelize the process by following the above mentioned repository.