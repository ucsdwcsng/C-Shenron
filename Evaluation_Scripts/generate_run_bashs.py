def create_run_eval_bash(eval_file, job_number):
    job_file = f'/radar-imaging-dataset/carla-radarimaging/Evaluation_Scripts/Start_Carla_Job_Scripts/job{job_number}.sh'
    qsub_template = f"""#!/bin/bash
cd /radar-imaging-dataset/carla-radarimaging/
echo 'inside /radar-imaging-dataset/carla-radarimaging/...' 
echo 'Installing the required packages...'
bash install_requirements.sh
echo 'Installation Done'

sleep 3s

dt=$(date '+%d/%m/%Y %H:%M:%S');
echo "Job started: $dt"

# Define the port number Carla is expected to listen on
CARLA_PORT=4321
echo 'starting carla...'
SDL_VIDEODRIVER=offscreen SDL_HINT_CUDA_DEVICE=0 ./../carla_garage_radar/carla/CarlaUE4.sh --carla-world-port=15180 -opengl -nosound -carla-streaming-port=$CARLA_PORT -quality-level=Low & 

CARLA_UP=0

while [ $CARLA_UP -eq 0 ]
do
    # Check if the port is open
    if netstat -tuln | grep ":$CARLA_PORT" > /dev/null; then
        echo "Carla client is up and listening on port $CARLA_PORT"
        CARLA_UP=1
    else
        echo "Carla client is still setting up to listen on port $CARLA_PORT"
        sleep 1m
    fi
done

echo "Loop finished"

echo 'carla started...'
sleep 3s

chmod u+x /radar-imaging-dataset/carla-radarimaging/Evaluation_Scripts/Job_Files/{eval_file}.sh

echo 'going to leaderboard evaluator bash...'
touch /radar-imaging-dataset/carla-radarimaging/Evaluation_Results/logs/{eval_file}.log
echo 'log file created...'
bash /radar-imaging-dataset/carla-radarimaging/Evaluation_Scripts/Job_Files/{eval_file}.sh > /radar-imaging-dataset/carla-radarimaging/Evaluation_Results/logs/{eval_file}.log 2>&1

sleep 2
"""

    with open(job_file, 'w', encoding='utf-8') as f:
        f.write(qsub_template)
    return job_file


def main():
    num_jobs = 10
    for i in range(num_jobs):
        eval_file = f'eval{i}'
        job_file = create_run_eval_bash(eval_file, i)
        print(f"Created job file: {job_file}")

# Main
if __name__=='__main__':
    main()