"""
Generates a dataset for training on a SLURM cluster.
Each route file is parallelized on its own machine.
Monitors the data collection and continues crashed processes.
Best run inside a tmux terminal.
"""

import subprocess
import time
from pathlib import Path
import os
import sys
import json
import fnmatch


def make_jobsub_file(commands, job_number, exp_name, dataset_name, root_folder, node, mail):
  # os.makedirs(f'{root_folder}{dataset_name}_logs/run_files/logs', exist_ok=True)
  # os.makedirs(f'{root_folder}{dataset_name}_logs/run_files/job_files', exist_ok=True)
  job_file = f'/radar-imaging-dataset/Pushkal/Data_Collection_Scripts/Start_Carla_Job_Scripts/job{job_number}.sh'
  qsub_template = f"""#!/bin/bash
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
"""
  for cmd in commands:
    qsub_template = qsub_template + f"""
{cmd}

"""

  with open(job_file, 'w', encoding='utf-8') as f:
    f.write(qsub_template)
  return job_file


def get_num_jobs(job_name, username, code_root):
  num_running_jobs = int(
      subprocess.check_output(
          f'SQUEUE_FORMAT2=\'username:{len(username)},name:130\' squeue --sort V | '
          f'grep {username} | grep {job_name} | wc -l',
          shell=True,
      ).decode('utf-8').replace('\n', ''))
  with open(f'{code_root}/max_num_jobs.txt', 'r', encoding='utf-8') as f:
    max_num_parallel_jobs = int(f.read())

  return num_running_jobs, max_num_parallel_jobs


# def get_carla_command(gpu, num_try, start_port, carla_root):
#   command = f'SDL_VIDEODRIVER=offscreen SDL_HINT_CUDA_DEVICE={int(gpu)} {carla_root}/CarlaUE4.sh ' \
#             f'--carla-world-port={int(gpu)*1000+start_port+num_try*10} -opengl -nosound -carla-streaming-port=0'
#   return command

# def get_carla_command(gpu, num_try, start_port, carla_root):
#   command = f'SDL_HINT_CUDA_DEVICE={int(gpu)} ./carla/CarlaUE4.sh ' \
#             f'--carla-world-port=15180 -opengl -nosound -carla-streaming-port=$CARLA_PORT -quality-level=Low &'
#   return command

# def get_command(gpu, num_try, start_port, carla_root):
#   command = f'chmod u+x /radar-imaging-dataset/carla_garage/run_bashs/run_autopilot_Town01_Scenario1_Repetition0.sh'
#   return command

def create_run_eval_bash(bash_save_dir, route_path, scenario_path, gpu, iteration, route, scen, repetition, dataset,
                         checkpoint, agent, start_port, carla_root, code_root, lib_path):
  with open(f'{bash_save_dir}/run_autopilot_{route}_Repetition{repetition}.sh', 'w', encoding='utf-8') as rsh:
    rsh.write(f'''\
#! /bin/bash
# CARLA path
export CARLA_ROOT=/radar-imaging-dataset/carla_garage/carla/
export WORK_DIR=/radar-imaging-dataset/carla_garage/

export CARLA_SERVER=${{CARLA_ROOT}}/CarlaUE4.sh
export PYTHONPATH=$PYTHONPATH:${{CARLA_ROOT}}/PythonAPI
export PYTHONPATH=$PYTHONPATH:${{CARLA_ROOT}}/PythonAPI/carla
export PYTHONPATH=$PYTHONPATH:${{CARLA_ROOT}}/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg

export SCENARIO_RUNNER_ROOT=${{WORK_DIR}}/scenario_runner
export LEADERBOARD_ROOT=${{WORK_DIR}}/leaderboard
export PYTHONPATH="${{CARLA_ROOT}}/PythonAPI/carla/":"${{SCENARIO_RUNNER_ROOT}}":"${{LEADERBOARD_ROOT}}":${{PYTHONPATH}}

# Server Ports
export PORT=15180 # same as the carla server port
export TM_PORT=25183 # port for traffic manager, required when spawning multiple servers/clients


# Evaluation Setup
export ROUTES={route_path}/{route}.xml
export SCENARIOS={scenario_path}/{scen}.json
export DEBUG_CHALLENGE=0 # visualization of waypoints and forecasting
export RESUME=1
export REPETITIONS=1
export DATAGEN=1
export BENCHMARK=collection
export GPU={str(gpu)}
export CHALLENGE_TRACK_CODENAME=MAP
# Agent Paths
export TEAM_AGENT="${{WORK_DIR}}/team_code/data_agent.py" # agent

    ''')
    rsh.write(f'''
export CHECKPOINT_ENDPOINT={dataset}/Routes_{route}_Repetition{repetition}/Dataset_generation_{route}_Repetition{repetition}.json # output results file
export SAVE_PATH={dataset}/Routes_{route}_Repetition{repetition} # path for saving episodes (comment to disable)
echo 'creating checkpoint and save_path directory...'
sleep 2s
mkdir -p ${{SAVE_PATH}}
touch ${{SAVE_PATH}}/Dataset_generation_{route}_Repetition{repetition}.json
echo 'running leaderboard_evaluator_local.py now...'
sleep 3s
        ''')
    rsh.write(f'''
CUDA_VISIBLE_DEVICES=${{GPU}} python3 ${{LEADERBOARD_ROOT}}/leaderboard/leaderboard_evaluator_local.py \
--scenarios=${{SCENARIOS}}  \
--routes=${{ROUTES}} \
--repetitions=${{REPETITIONS}} \
--track=${{CHALLENGE_TRACK_CODENAME}} \
--checkpoint=${{CHECKPOINT_ENDPOINT}} \
--agent=${{TEAM_AGENT}} \
--agent-config=${{TEAM_CONFIG}} \
--debug=${{DEBUG_CHALLENGE}} \
--record=${{RECORD_PATH}} \
--resume=${{RESUME}} \
--port=${{PORT}} \
--trafficManagerPort=${{TM_PORT}} \
--timeout=600.0
''')
    rsh.write(f'''
mkdir -p /radar-imaging-dataset/carla_garage_data/{dataset.split('/')[-1]}

cp -r /home/user/datagen_carla_garage/hb_dataset_v08_2023_05_10/{dataset.split('/')[-1]}/Routes_{route}_Repetition{repetition}/ /radar-imaging-dataset/carla_garage_data/{dataset.split('/')[-1]}/
''')


def main():
  carla_port = 15180
  code_root = r'/radar-imaging-dataset/Pushkal/'
  carla_root = r'/path/to/carla_9_10'
  data_routes_dir = code_root + r'/leaderboard/data/training'
  # Our centOS is missing some c libraries.
  # Usually miniconda has them, so we tell the linker to look there as well.
  lib_path = r'/path/to/miniconda/lib'
  date = '2024_09_09'
  dataset_name = 'hb_dataset_v08_' + date
  # root_folder = r'/path/to/data/'  # With ending slash
  root_folder = r'/radar-imaging-dataset/Pushkal/datagen_carla_garage/'  # With ending slash
  data_save_root = root_folder + dataset_name

  log_root = root_folder + dataset_name + '_logs'
  num_repetitions = 1
  node = 'slurm-partition'
  username = 'slurm-username'
  mail = 'your-email'
  exp_name = 'exp_name_'
  
  route_paths = []
  save_folders = []
  repetitions = []
  for i in range(num_repetitions):
    # route_paths.append(f'{data_routes_dir}/routes/s1')
    # route_paths.append(f'{data_routes_dir}/routes/s3')
    # route_paths.append(f'{data_routes_dir}/routes/s4')
    # route_paths.append(f'{data_routes_dir}/routes/s7')
    route_paths.append(f'{data_routes_dir}/routes/s8')
    # route_paths.append(f'{data_routes_dir}/routes/s9')
    # route_paths.append(f'{data_routes_dir}/routes/s10')
    # route_paths.append(f'{data_routes_dir}/routes/ll')
    # route_paths.append(f'{data_routes_dir}/routes/lr')
    # route_paths.append(f'{data_routes_dir}/routes/rl')
    # route_paths.append(f'{data_routes_dir}/routes/rr')

    # save_folders.append(f'{data_save_root}/s1_dataset_' + date)
    # save_folders.append(f'{data_save_root}/s3_dataset_' + date)
    # save_folders.append(f'{data_save_root}/s4_dataset_' + date)
    # save_folders.append(f'{data_save_root}/s7_dataset_' + date)
    save_folders.append(f'{data_save_root}/s8_dataset_' + date)
    # save_folders.append(f'{data_save_root}/s9_dataset_' + date)
    # save_folders.append(f'{data_save_root}/s10_dataset_' + date)
    # save_folders.append(f'{data_save_root}/ll_dataset_' + date)
    # save_folders.append(f'{data_save_root}/lr_dataset_' + date)
    # save_folders.append(f'{data_save_root}/rl_dataset_' + date)
    # save_folders.append(f'{data_save_root}/rr_dataset_' + date)

    for _ in range(11):
      repetitions.append(i)

  gpu = 0
  iteration = 0
  job_nr = 0

  route_files = []
  for i in range(len(route_paths)):
    route_path = route_paths[i]

    route_pattern = '*.xml'

    for root, _, files in os.walk(route_path):

      for name in files:
        if fnmatch.fnmatch(name, route_pattern):
          route_files.append(os.path.join(root, name))

  num_jobs = len(route_files)

  meta_jobs = {}
  print('Starting to generate data')
  print(f'Number of jobs: {num_jobs}')

  for i in range(len(route_paths)):
    dataset = f'{save_folders[i]}'
    route_path = route_paths[i]
    repetition = repetitions[i]

    route_pattern = '*.xml'
    checkpoint = ''
    agent = 'data_agent'

    route_files = []
    scen_files = []

    for root, _, files in os.walk(route_path):
      for name in files:
        if fnmatch.fnmatch(name, route_pattern):
          route_files.append(os.path.join(root, name))
          if root.split('/')[-1].startswith('s'):
            scenario = root.split('/')[-1].split('s')[-1]
            town = name.split('Town')[-1][:2]
            if town == '10':
              town = '10HD'
            if os.path.exists(f'{data_routes_dir}/scenarios/s{scenario}/Town{town}_Scenario{scenario}.json'):
              scen_files.append(f'{data_routes_dir}/scenarios/s{scenario}/Town{town}_Scenario{scenario}.json')
            elif os.path.exists(os.path.join(root, f'/Town{town}_30m_Scenario{scenario}.json')):
              scen_files.append(os.path.join(root, f'/Town{town}_30m_Scenario{scenario}.json'))
            else:
              print(f'No scenario file for {root}')
              sys.exit()
          else:
            assert os.path.exists(f'{data_routes_dir}/scenarios/eval_scenarios.json'
                                 ), f'{f"{data_routes_dir}/scenarios/eval_scenarios.json"} does not exist'
            scen_files.append(f'{data_routes_dir}/scenarios/eval_scenarios.json')

    # bash_save_dir = Path(f'{log_root}/run_bashs')
    bash_save_dir = '/radar-imaging-dataset/Pushkal/Data_Collection_Scripts/Job_Files/'
    # bash_save_dir.mkdir(parents=True, exist_ok=True)

    for ix, route in enumerate(route_files):
      route = Path(route).stem
      scen = Path(scen_files[ix]).stem
      scenario_path = Path(scen_files[ix]).parent.absolute()

      results_save_dir = Path(f'{dataset}/Routes_{route}_Repetition{repetition}')
      results_save_dir.mkdir(parents=True, exist_ok=True)

      commands = []

      # carla_cmd = get_command(gpu=gpu, num_try=iteration, start_port=carla_port, carla_root=carla_root)
      # commands.append(f'{carla_cmd} &')
      commands.append(f'echo going to leaderboard evaluator bash...')
      
      
      
      create_run_eval_bash(bash_save_dir=bash_save_dir,
                           route_path=route_path,
                           scenario_path=scenario_path,
                           gpu=gpu,
                           iteration=iteration,
                           route=route,
                           scen=scen,
                           repetition=repetition,
                           dataset=dataset,
                           checkpoint=checkpoint,
                           agent=agent,
                           start_port=carla_port,
                           carla_root=carla_root,
                           code_root=code_root,
                           lib_path=lib_path)
      
      
      # commands.append(f'/radar-imaging-dataset/carla_garage/run_bashs/run_autopilot_Town01_Scenario1_Repetition0.sh')
      # commands.append(f'sleep 2')
      
      
      commands.append(f'chmod u+x {bash_save_dir}run_autopilot_{route}_Repetition{repetition}.sh')
      commands.append(f'{bash_save_dir}run_autopilot_{route}_Repetition{repetition}.sh')
      commands.append('sleep 2')
      iteration += 1

      job_file = make_jobsub_file(commands=commands,
                                  job_number=job_nr,
                                  exp_name=exp_name,
                                  dataset_name=dataset_name,
                                  root_folder=root_folder,
                                  node=node,
                                  mail=mail)
      # Wait until submitting new jobs that the #jobs are at below max
      
      
      num_running_jobs, max_num_parallel_jobs = get_num_jobs(job_name=exp_name, username=username, code_root=code_root)
      # print(f'{num_running_jobs}/{max_num_parallel_jobs} jobs are running...')
      # while num_running_jobs >= max_num_parallel_jobs:
        # num_running_jobs, max_num_parallel_jobs = get_num_jobs(job_name=exp_name,
        #                                                        username=username,
        #                                                        code_root=code_root)
        # time.sleep(5)
      print(f'Submitting job {job_nr}/{num_jobs}: {job_file}')
      job_nr += 1
      # break
      # result_file = f'{results_save_dir}/Dataset_generation_{route}_Repetition{repetition}.json'
      # jobid = subprocess.check_output(f'sbatch {job_file}', shell=True).decode('utf-8').strip().rsplit(' ',
                                                                                                      #  maxsplit=1)[-1]
      # meta_jobs[jobid] = (False, job_file, result_file, 0)
      # time.sleep(0.2)  # because of automatic carla port assignment

  # time.sleep(380)
  # training_finished = False
  
  # while not training_finished:
  #   num_running_jobs, max_num_parallel_jobs = get_num_jobs(job_name=exp_name, username=username, code_root=code_root)
  #   print(f'{num_running_jobs} jobs are running... Job: {exp_name}')
  #   time.sleep(20)

  #   # resubmit unfinished jobs
  #   for k in list(meta_jobs.keys()):
  #     job_finished, job_file, result_file, resubmitted = meta_jobs[k]
  #     need_to_resubmit = False
  #     if not job_finished and resubmitted < 10:
  #       # check whether job is running
  #       if int(subprocess.check_output(f'squeue | grep {k} | wc -l', shell=True).decode('utf-8').strip()) == 0:
  #         # check whether result file is finished?
  #         if os.path.exists(result_file):
  #           with open(result_file, 'r', encoding='utf-8') as f_result:
  #             evaluation_data = json.load(f_result)
  #           progress = evaluation_data['_checkpoint']['progress']
  #           if len(progress) < 2 or progress[0] < progress[1]:
  #             need_to_resubmit = True
  #           else:
  #             for record in evaluation_data['_checkpoint']['records']:
  #               if record['scores']['score_route'] <= 0.00000000001:
  #                 need_to_resubmit = True
  #               if record['status'] == 'Failed - Agent couldn\'t be set up':
  #                 need_to_resubmit = True
  #               if record['status'] == 'Failed':
  #                 need_to_resubmit = True
  #               if record['status'] == 'Failed - Simulation crashed':
  #                 need_to_resubmit = True
  #               if record['status'] == 'Failed - Agent crashed':
  #                 need_to_resubmit = True

  #           if not need_to_resubmit:
  #             # delete old job
  #             print(f'Finished job {job_file}')
  #             meta_jobs[k] = (True, None, None, 0)

  #         else:
  #           need_to_resubmit = True

  #     if need_to_resubmit:
  #       print(f'resubmit sbatch {job_file}')

  #       # rename old error files to still access it
  #       job_nr_log = Path(job_file).stem
  #       time_now_log = time.time()
  #       os.system(f'mkdir -p "{log_root}/run_files/logs_{time_now_log}"')
  #       os.system(f'cp {log_root}/run_files/logs/qsub_err{job_nr_log}.log {log_root}/run_files/logs_{time_now_log}')
  #       os.system(f'cp {log_root}/run_files/logs/qsub_out{job_nr_log}.log {log_root}/run_files/logs_{time_now_log}')

  #       jobid = subprocess.check_output(f'sbatch {job_file}', shell=True).decode('utf-8').strip().rsplit(' ',
  #                                                                                                        maxsplit=1)[-1]
  #       meta_jobs[jobid] = (False, job_file, result_file, resubmitted + 1)
  #       meta_jobs[k] = (True, None, None, 0)

  #   time.sleep(10)

  #   if num_running_jobs == 0:
  #     training_finished = True


if __name__ == '__main__':
  main()
