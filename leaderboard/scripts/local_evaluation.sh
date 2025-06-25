export CARLA_ROOT=${1:-/radar-imaging-dataset/carla_garage_radar/carla}
export WORK_DIR=${2:-/radar-imaging-dataset/carla_garage_radar}

export CARLA_SERVER=${CARLA_ROOT}/CarlaUE4.sh
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla
export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg
export SCENARIO_RUNNER_ROOT=${WORK_DIR}/scenario_runner
export LEADERBOARD_ROOT=${WORK_DIR}/leaderboard
export PYTHONPATH="${CARLA_ROOT}/PythonAPI/carla/":"${SCENARIO_RUNNER_ROOT}":"${LEADERBOARD_ROOT}":${PYTHONPATH}

export SCENARIOS=${WORK_DIR}/leaderboard/data/scenarios/eval_scenarios.json
# export SCENARIOS=${WORK_DIR}/leaderboard/data/scenarios/no_scenarios.json

# export ROUTES=${WORK_DIR}/leaderboard/data/longest6.xml
# export ROUTES=${WORK_DIR}/leaderboard/data/routes_eval_NEAT.xml
export ROUTES=${WORK_DIR}/leaderboard/data/routes_eval_NEAT_id_0.xml
export REPETITIONS=1
export CHALLENGE_TRACK_CODENAME=SENSORS
# export CHECKPOINT_ENDPOINT=${WORK_DIR}/results/transfuser_plus_plus_NEAT.json
export CHECKPOINT_ENDPOINT=${WORK_DIR}/result_ours/NEAT_IMAGE_RADAR_FB_cat_re0.5/tf_cr_31.json
export TEAM_AGENT=${WORK_DIR}/team_code/sensor_agent.py
# export TEAM_CONFIG=${WORK_DIR}/model_ckpt/pretrained_models/longest6/tfpp_all_0
# export TEAM_CONFIG=/radar-imaging-dataset/carla_garage_logdir/train_id_007 #(the lidar working model)
# export TEAM_CONFIG=/radar-imaging-dataset/carla_garage_logdir/train_id_radar_fbCh_model
export TEAM_CONFIG=/radar-imaging-dataset/carla_garage_logdir/train_id_radar_fb_cat_model
export DB_ON=0
export RADAR_CHANNEL=1
export RADAR_CAT=1 #this should be one only when the fb cat view is required
export RADAR_FBLR=0
export BLACKOUT_RADAR=0
export BLACKOUT_IMAGE=0
export DEBUG_CHALLENGE=0
export RESUME=1
export DATAGEN=0
export EVAL_DATAGEN=0
export SAVE_PATH=${WORK_DIR}/result_ours/NEAT_IMAGE_RADAR_FB_cat_re0.5
export UNCERTAINTY_THRESHOLD=0.33
export BENCHMARK=longest6
export RECORD_PATH='/radar-imaging-dataset/carla_garage_radar/result_ours/NEAT_IMAGE_RADAR_FB_cat_re0.5/recordings/' 
#create the recordings directory before running the program

echo 'copying the local_evaluation.sh file to save path'
cp ./local_evaluation.sh $SAVE_PATH/
echo 'done copying the config file'

echo 'going to leaderboard local evaluator.py'
python3 ${LEADERBOARD_ROOT}/leaderboard/leaderboard_evaluator_local.py \
--scenarios=${SCENARIOS}  \
--routes=${ROUTES} \
--repetitions=${REPETITIONS} \
--track=${CHALLENGE_TRACK_CODENAME} \
--checkpoint=${CHECKPOINT_ENDPOINT} \
--agent=${TEAM_AGENT} \
--agent-config=${TEAM_CONFIG} \
--debug=0 \
--resume=${RESUME} \
--timeout=600 \
--record=${RECORD_PATH}
