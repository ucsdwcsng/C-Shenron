#!/bin/bash
cd /radar-imaging-dataset/Pushkal/
echo 'inside /radar-imaging-dataset/Pushkal/...' 
echo 'Installing the required packages...'
bash install_requirements.sh
echo 'Installation Done'

sleep 3s

dt=$(date '+%d/%m/%Y %H:%M:%S');
echo "Job started: $dt"

# Define the port number Carla is expected to listen on
CARLA_PORT=4321
echo 'starting carla...'
SDL_VIDEODRIVER=offscreen SDL_HINT_CUDA_DEVICE=0 /radar-imaging-dataset/carla_garage_radar/carla/CarlaUE4.sh --carla-world-port=15180 -opengl -nosound -carla-streaming-port=$CARLA_PORT -quality-level=Low & 

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

echo going to leaderboard evaluator bash...


chmod u+x /radar-imaging-dataset/Pushkal/Data_Collection_Scripts/Job_Files/run_autopilot_Town07_Scenario8_Repetition0.sh


/radar-imaging-dataset/Pushkal/Data_Collection_Scripts/Job_Files/run_autopilot_Town07_Scenario8_Repetition0.sh


sleep 2

