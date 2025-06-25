import os
def make_job_file(job_number, eval_model, results_file):
    job_file = f'/radar-imaging-dataset/carla-radarimaging/Evaluation_Scripts/Job_Files/eval{job_number}.sh' #path where the job file be saved with job_number
    
    job_temp = f"""export CARLA_ROOT=${{1:-/radar-imaging-dataset/carla_garage_radar/carla}}
export WORK_DIR=${{2:-/radar-imaging-dataset/carla-radarimaging}}

export CARLA_SERVER=${{CARLA_ROOT}}/CarlaUE4.sh
export PYTHONPATH=$PYTHONPATH:${{CARLA_ROOT}}/PythonAPI
export PYTHONPATH=$PYTHONPATH:${{CARLA_ROOT}}/PythonAPI/carla
export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg
export SCENARIO_RUNNER_ROOT=${{WORK_DIR}}/scenario_runner
export LEADERBOARD_ROOT=${{WORK_DIR}}/leaderboard
export PYTHONPATH="${{CARLA_ROOT}}/PythonAPI/carla/":"${{SCENARIO_RUNNER_ROOT}}":"${{LEADERBOARD_ROOT}}":${{PYTHONPATH}}

# Server Ports
export PORT=15180 # same as the carla server port
export TM_PORT=25183 # port for traffic manager, required when spawning multiple servers/clients

export SCENARIOS=${{WORK_DIR}}/leaderboard/data/scenarios/eval_scenarios.json

# export ROUTES=${{WORK_DIR}}/leaderboard/data/longest6.xml
# export ROUTES=${{WORK_DIR}}/leaderboard/data/routes_eval_NEAT.xml
export ROUTES=${{WORK_DIR}}/leaderboard/data/lav.xml
export REPETITIONS=1
export CHALLENGE_TRACK_CODENAME=SENSORS
export CHECKPOINT_ENDPOINT=${{WORK_DIR}}/Evaluation_Results/{results_file}/Iteration{job_number}/Test.json
export TEAM_AGENT=${{WORK_DIR}}/team_code/sensor_agent.py
export TEAM_CONFIG=/radar-imaging-dataset/carla-radarimaging/carla_garage_logdir/{eval_model}
# export TEAM_CONFIG=/radar-imaging-dataset/carla_garage_logdir/train_id_radar_fb+lr_cat_86v0.2_model
export DB_ON=0
export RADAR_CHANNEL=1
export RADAR_CAT=2 #this should be one only when the fb cat view is required
export BLACKOUT_RADAR=0
export BLACKOUT_IMAGE=0
export DEBUG_CHALLENGE=0
export RESUME=1
export DATAGEN=0
export EVAL_DATAGEN=0
export SAVE_PATH=${{WORK_DIR}}/Evaluation_Results/{results_file}/Iteration{job_number}
export UNCERTAINTY_THRESHOLD=0.33
export BENCHMARK=longest6
export RECORD_PATH='/radar-imaging-dataset/carla-radarimaging/Evaluation_Results/{results_file}/Iteration{job_number}/recordings/' 
mkdir -p $RECORD_PATH
#create the recordings directory before running the program

echo 'copying the local_evaluation.sh file to save path'
cp {job_file} $SAVE_PATH/
echo 'done copying the config file'

echo 'going to leaderboard local evaluator.py'
python3 ${{LEADERBOARD_ROOT}}/leaderboard/leaderboard_evaluator_local.py --scenarios=${{SCENARIOS}} --routes=${{ROUTES}} --repetitions=${{REPETITIONS}} --track=${{CHALLENGE_TRACK_CODENAME}} --checkpoint=${{CHECKPOINT_ENDPOINT}} --agent=${{TEAM_AGENT}} --agent-config=${{TEAM_CONFIG}} --debug=0 --resume=${{RESUME}} --record=${{RECORD_PATH}} --port=${{PORT}} --trafficManagerPort=${{TM_PORT}} --timeout=180.0
"""

#write this to a file
    with open(job_file, 'w', encoding='utf-8') as f:
        f.write(job_temp)
    return job_file


def main():
    eval_model = "new_fblr_cat_86_v3_model"
    #concat an iterator to it based on the number of jobs to create
    results_file = "LAV_Evaluations_new_fblr" # all the logs, scores and recordings will be saved inside this file
    routes = "" # hardcoded for now, since all the pods will run the eval on same eval routes
    
    num_jobs = 10 # number of jobs to create
    for i in range(num_jobs):
        job_file = make_job_file(i, eval_model, results_file)
        print(f"Created job file: {job_file}")

if __name__=='__main__':
    main()
