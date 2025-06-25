# -*- coding: utf-8 -*-
"""
Created on Wed Apr  7 16:00:26 2021

@author: ksban
"""

from lidar import run_lidar
import os
import yaml

def run_shenron(sim_config, base_folder):
    print("in run shenron")
    # with open('simulator_configs.yaml', 'r') as f:
    #     sim_config = yaml.safe_load(f)
    
    # printing the base path here for the check
    # print(f"the base folder currently is base/")

    folders = os.listdir(f"{sim_config['BASE_PATH']}/{base_folder}")
    folders.sort()
    # print(folders)
    # run_folders = ["set4_vertical_crossing_2023-10-08-13-07-48", "set3_split_vertical_crossing_2023-10-08-12-56-58", "set2_vertical_crossing_2023-10-08-12-24-52", "set1_vertical_crossing_2023-10-08-12-02-41"]
    # return
    # run_folders= ["Town01_20_08_10_09_47_16"] 
    # run_folders= ["Town01_10_08_10_06_43_59"] #with 3 chirps only
    for folder in folders:
        # if folder in run_folders:
        if os.path.isdir(f'{sim_config["BASE_PATH"]}/{base_folder}/{folder}/'):
            exec_path = f'{sim_config["BASE_PATH"]}/{base_folder}/{folder}/'
            print(f"currently running the folder {exec_path.split('/')[-2]}")
            if sim_config["INPUT"] == "lidar":
                run_lidar(sim_config, exec_path)
            else:
                print("Incorrect input in config")
            
            print("this folder done")

        else:
            print("skipping this, not a folder")
            continue
        
        # else:
        #     continue
        
if __name__ == '__main__':

    with open('simulator_configs.yaml', 'r') as f:
        sim_config = yaml.safe_load(f)

    # if sim_config["INPUT"] == "lidar":
    #     run_lidar(sim_config)
    # else:
    #     print("Incorrect input in config")
    print("in main")

    base_base_folders = os.listdir(sim_config["BASE_PATH"])
    base_base_folders.sort()
    run_folders = ["Town05_tiny", "Town06_short", "Town06_tiny", "Town07_short", "Town07_tiny", "Town10_short", "Town10_tiny",]
    for base_folder in base_base_folders:
        if base_folder in run_folders:
            print(f"currently running the {base_folder}")
            run_shenron(sim_config, base_folder)