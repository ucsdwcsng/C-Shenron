# Parallelize workflows using nautilus

A lot of times in our work we need to process large chunks of data. Often this can be broken into smaller parallel tasks. For example in one of our workflows we had to convert thousands of lidar files into radar data using our simulator. Clearly, processing of each lidar file is independent of the other file. These kind of workloads are called [Emabarrasingly Parallel](https://en.wikipedia.org/wiki/Embarrassingly_parallel). 

If you identify your task to fall into this category, nautilus is your biggest friend. In the lidar to radar conversion example, we were able to bring down the processing time from 6-7 days to overnight. This is insane optimization. In this document we have described how to do it for your own task.

## Identifying the unit of parallelization

The most important step is to indentify at what level you would like to perform parallelization. In our example, we divided the data into several folders based on the different times they were collected at. You need to make sure that each of these folders can be operated on parallely. Lets call each of these smaller tasks as *subtasks*. 

Note that every time you run a subtask, it might have an overhead of its own, like some library installation etc., which would cause some delay. So you should divide your task in a way where the overhead is 5% or less of the total time spent in each subtask.

## Creating and running the jobs for subtasks

### Overview

Once you have identified the unit of parallelization, each subtask would run as a separate job. In order to so this, we need to first create a shell script per subtask, that can run that subtask. Then, we need to create yaml file that can launch a job on kubernetes, which will run a subtask. Multiple jobs can run at the same time, hence achieving parallelization. Please refer to xxxx for definitions of yaml and jobs.

### On kubernetes/nautilus side -- Creating Shell Scripts

1. This first step is to create individual shell scripts per subtask. An example script of generating shell scripts is [data_generation_bash_scripts.py](data_generation_bash_scripts.py).
 
2. [data_generation_bash_scripts.py](data_generation_bash_scripts.py) simply iterates over the list of all folders that we created for our subtasks and create shell scripts for each one of them. An example shell script is [job0.sh](Data_Collection_Scripts/Start_Carla_Job_Scripts/job0.sh). This script first installs a few libraries in the environment and then run the subtask. Each subtask operates on a unique folder, which is specified in the shell script.

3. Make sure that these shell scripts for subtasks would reside on your **pvc** in your cluster. We will specify this pvc when we would launch jobs for each subtask. In the given example, we have them in `pvcvols` pvc.


### On local machine

1. You need to create `.yaml` files that will run the jobs. Create a folder named `jobs` (the name is not important). An example script to generate yaml files is [generate_job_yaml.py](parallelization/generate_job_yaml.py). 

2. In this file we specify `MAX_JOBS` as the total number of jobs we want to create. This should match the total number of jobs you created in previous step.

3. You can specify custom names for your jobs.

4. **Line 26** is where we specify the command that will run when we launch this job. It should change the directory to the location where your shell scripts reside.

5. **Line 53** specify the memory, cpu and gpu requests.

6. Make sure to fill in additional details of your cluster in the generate_yaml function.

7. Once you run [generate_job_yaml.py](parallelization/generate_job_yaml.py), all the yaml files will get generated in the `jobs` folder.

8. Finally run [create_jobs.sh](parallelization/create_jobs.sh). This file simply runs the following command for all the yaml files
```
kubectl create -f <yaml file name>
```

9. Once the jobs are over, you can delete all the jobs by running the [kill_jobs.sh](parallelization/kill_jobs.sh) script. This will delete all the jobs that were created by the `create_jobs.sh` script.

## Final remark
Note, all the above steps are simply an example of how we did parallelization. The main concept is to divide your task into subtasks and launch them as separate jobs. You can write your own scripts to automoate that process.

## Credits
This document was made by the help of Kshitiz Bansal.