#! /bin/bash
# CARLA path
export CARLA_ROOT=/radar-imaging-dataset/carla_garage/carla/
export WORK_DIR=/radar-imaging-dataset/carla_garage/

export CARLA_SERVER=${CARLA_ROOT}/CarlaUE4.sh
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg

export SCENARIO_RUNNER_ROOT=${WORK_DIR}/scenario_runner
export LEADERBOARD_ROOT=${WORK_DIR}/leaderboard
export PYTHONPATH="${CARLA_ROOT}/PythonAPI/carla/":"${SCENARIO_RUNNER_ROOT}":"${LEADERBOARD_ROOT}":${PYTHONPATH}

# Server Ports
export PORT=15180 # same as the carla server port
export TM_PORT=25183 # port for traffic manager, required when spawning multiple servers/clients


# Evaluation Setup
export ROUTES=/radar-imaging-dataset/Pushkal//leaderboard/data/training/routes/s8/Town04_Scenario8.xml
export SCENARIOS=/radar-imaging-dataset/Pushkal/leaderboard/data/training/scenarios/s8/Town04_Scenario8.json
export DEBUG_CHALLENGE=0 # visualization of waypoints and forecasting
export RESUME=1
export REPETITIONS=1
export DATAGEN=1
export BENCHMARK=collection
export GPU=0
export CHALLENGE_TRACK_CODENAME=MAP
# Agent Paths
export TEAM_AGENT="${WORK_DIR}/team_code/data_agent.py" # agent

    
export CHECKPOINT_ENDPOINT=/radar-imaging-dataset/Pushkal/datagen_carla_garage/hb_dataset_v08_2024_09_09/s8_dataset_2024_09_09/Routes_Town04_Scenario8_Repetition0/Dataset_generation_Town04_Scenario8_Repetition0.json # output results file
export SAVE_PATH=/radar-imaging-dataset/Pushkal/datagen_carla_garage/hb_dataset_v08_2024_09_09/s8_dataset_2024_09_09/Routes_Town04_Scenario8_Repetition0 # path for saving episodes (comment to disable)
echo 'creating checkpoint and save_path directory...'
sleep 2s
mkdir -p ${SAVE_PATH}
touch ${SAVE_PATH}/Dataset_generation_Town04_Scenario8_Repetition0.json
echo 'running leaderboard_evaluator_local.py now...'
sleep 3s
        
CUDA_VISIBLE_DEVICES=${GPU} python3 ${LEADERBOARD_ROOT}/leaderboard/leaderboard_evaluator_local.py --scenarios=${SCENARIOS}  --routes=${ROUTES} --repetitions=${REPETITIONS} --track=${CHALLENGE_TRACK_CODENAME} --checkpoint=${CHECKPOINT_ENDPOINT} --agent=${TEAM_AGENT} --agent-config=${TEAM_CONFIG} --debug=${DEBUG_CHALLENGE} --record=${RECORD_PATH} --resume=${RESUME} --port=${PORT} --trafficManagerPort=${TM_PORT} --timeout=600.0

mkdir -p /radar-imaging-dataset/carla_garage_data/s8_dataset_2024_09_09

cp -r /home/user/datagen_carla_garage/hb_dataset_v08_2023_05_10/s8_dataset_2024_09_09/Routes_Town04_Scenario8_Repetition0/ /radar-imaging-dataset/carla_garage_data/s8_dataset_2024_09_09/
