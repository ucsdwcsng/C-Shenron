#!/bin/bash

bash ./../install_requirements.sh

export CARLA_ROOT=/radar-imaging-dataset/carla_garage_radar/carla
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg
export PYTHONPATH="${CARLA_ROOT}/PythonAPI/carla/":${PYTHONPATH}
# export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/path/to/miniconda3/lib

export OMP_NUM_THREADS=20  # Limits pytorch to spawn at most num cpus cores threads
export OPENBLAS_NUM_THREADS=1  # Shuts off numpy multithreading, to avoid threads spawning other threads.
export TRAIN_ID=new_fblr_entire_data_training

touch /radar-imaging-dataset/carla-radarimaging/carla_garage_logdir/training_logs/$TRAIN_ID.log
echo 'created log file...'

torchrun --nnodes=1 --nproc_per_node=gpu --max_restarts=1 --rdzv_id=42353467 --rdzv_backend=c10d train.py --id $TRAIN_ID \
    --epochs 30 \
    --batch_size 12 \
    --setting all \
    --root_dir /radar-imaging-dataset/carla_garage_data \
    --logdir /radar-imaging-dataset/carla-radarimaging/carla_garage_logdir \
    --use_controller_input_prediction 1 \
    --use_wp_gru 0 \
    --use_discrete_command 1 \
    --use_tp 1 \
    --continue_epoch 0 \
    --cpu_cores 64 \
    --num_repetitions 3 \
    --use_disk_cache 1 \
    --radar_channels 1 \
    --radar_cat 2    > /radar-imaging-dataset/carla-radarimaging/carla_garage_logdir/training_logs/$TRAIN_ID.log
    # radar_cat 1 will concat only the front and back view whereas 2 will concat fb+lr.
    # batch_size was initially 12, changing due to 8 gpu issue

# torchrun --nnodes=1 --nproc_per_node=8 --max_restarts=1 --rdzv_id=42353467 --rdzv_backend=c10d train.py --id train_id_000 --batch_size 8 --setting 02_05_withheld --root_dir /path/to/dataset --logdir /path/to/logdir --use_controller_input_prediction 1 --use_wp_gru 0 --use_discrete_command 1 --use_tp 1 --continue_epoch 1 --cpu_cores 20 --num_repetitions 3
# torchrun --nnodes=1 --nproc_per_node=gpu --max_restarts=1 --rdzv_id=42353467 --rdzv_backend=c10d train.py --id train_id_radar_with_cos \
