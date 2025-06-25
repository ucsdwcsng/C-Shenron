import numpy as np
import os
import yaml
# import run_shenron
# from main import run_shenron

def check_save_path(save_path):
    if not os.path.exists(save_path):
        os.makedirs(save_path)
    return

def main():
    
    with open('carla_shenron_config.yaml', 'r') as f:
        config = yaml.safe_load(f)
    
    in_path = config["INPUT_PATH"]
    out_path = config["OUTPUT_PATH"]

    if not os.path.exists(out_path):
        os.makedirs(out_path, exist_ok=True)

    folders = os.listdir(in_path)
    folders.sort()
    print(folders)
    print("folders loaded, mapping now")

    for folder in folders:
        if os.path.isdir(f'{in_path}/{folder}'):
            print(f"starting {folder}")
            out_path_temp = f'{out_path}/{folder}/carla_shenron_sem_lidar'
            check_save_path(out_path_temp)
            
            files = os.listdir(f'{in_path}/{folder}/semantic_lidar/')
            files.sort()

            for file in files:
                carla_sem_lidar_data = np.load(f'{in_path}/{folder}/semantic_lidar/{file}')
                carla_sem_lidar_data = carla_sem_lidar_data[:, (0, 1, 2, 5)]
                carla_sem_lidar_data[:, 3] = carla_sem_lidar_data[:, 3]-1
                
                carla_sem_lidar_data[carla_sem_lidar_data[:, 3]>18, 3] = 255.
                carla_sem_lidar_data[carla_sem_lidar_data[:, 3]<0, 3] = 255.
                # print(carla_sem_lidar_data)
                carla_sem_lidar_data[:, (0, 1, 2)] = carla_sem_lidar_data[:, (0, 2, 1)]
                # print(carla_sem_lidar_data)
                # break
                # print(np.unique(carla_sem_lidar_data[:, 3]))
                # break
                np.save(f'{out_path_temp}/{file[:-4]}.npy', carla_sem_lidar_data)
            print(f"{folder} done")
        else:
            print("skipping this, not a directory")
            continue

    #call shenron here after saving carla_shenron_semantic_lidar for all the folders
    # print("runnning shenron now")
    # run_shenron()


if __name__ == "__main__":
    print("starting map_carla_semantic_lidar_shenron")
    main()