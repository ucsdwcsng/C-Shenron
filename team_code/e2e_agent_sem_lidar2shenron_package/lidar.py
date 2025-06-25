import sys
import os
# sys.path.append("../")
sys.path.append('radar-imaging-dataset/carla_garage_radar/team_code/e2e_agent_sem_lidar2shenron_package/')

import numpy as np
from e2e_agent_sem_lidar2shenron_package.path_config import *
from e2e_agent_sem_lidar2shenron_package.ConfigureRadar import radar
from e2e_agent_sem_lidar2shenron_package.shenron.Sceneset import *
from e2e_agent_sem_lidar2shenron_package.shenron.heatmap_gen_fast import *
# from pointcloud_raytracing.pointraytrace import ray_trace
import scipy.io as sio
from e2e_agent_sem_lidar2shenron_package.lidar_utils import *
import time
import shutil
import pdb

def map_carla_semantic_lidar_latest(carla_sem_lidar_data):
    '''
    Function to map material column in the collected carla ray_cast_shenron to shenron input
    '''
    # print(carla_sem_lidar_data.shape())
    carla_sem_lidar_data_crop = carla_sem_lidar_data[:, (0, 1, 2, 5)]
    temp_list = np.array([0, 4, 2, 0, 11, 5, 0, 0, 1, 8, 12, 3, 7, 10, 0, 1, 0, 12, 6, 0, 0, 0, 0])

    col = temp_list[(carla_sem_lidar_data_crop[:, 3].astype(int))]
    carla_sem_lidar_data_crop[:, 3] = col

    return carla_sem_lidar_data_crop

# def map_carla_semantic_lidar(carla_sem_lidar_data):
#     '''
#     Function to map material column in the collected carla ray_cast_shenron to shenron input 
#     '''
#     # print(carla_sem_lidar_data.shape())
#     carla_sem_lidar_data_crop = carla_sem_lidar_data[:, (0, 1, 2, 5)]
#     carla_sem_lidar_data_crop[:, 3] = carla_sem_lidar_data_crop[:, 3]-1
#     carla_sem_lidar_data_crop[carla_sem_lidar_data_crop[:, 3]>18, 3] = 255.
#     carla_sem_lidar_data_crop[carla_sem_lidar_data_crop[:, 3]<0, 3] = 255.
#     carla_sem_lidar_data_crop[:, (0, 1, 2)] = carla_sem_lidar_data_crop[:, (0, 2, 1)]
#     return carla_sem_lidar_data_crop

def check_save_path(save_path):
    if not os.path.exists(save_path):
        os.makedirs(save_path)
    return

def rotate_points(points, angle):
    rotMatrix = np.array([[np.cos(np.deg2rad(angle)), np.sin(np.deg2rad(angle)), 0]
        , [- np.sin(np.deg2rad(angle)), np.cos(np.deg2rad(angle)), 0]
        , [0, 0, 1]])
    return np.matmul(points, rotMatrix)

def Cropped_forRadar(pc, veh_coord, veh_angle, radarobj):
    """
    Removes Occlusions and calculates loss for each point
    """

    skew_pc = rotate_points(pc[:, 0:3] , veh_angle )
    # skew_pc = np.vstack(((skew_pc ).T, pc[:, 3], pc[:, 5])).T  #x,y,z,speed,material
    skew_pc = np.vstack(((skew_pc ).T, pc[:, 3], pc[:, 5],pc[:,6])).T  #x,y,z,speed,material, cosines

    rowy = np.where((skew_pc[:, 1] > 0.8))
    new_pc = skew_pc[rowy, :].squeeze(0)

    new_pc = new_pc[new_pc[:,4]!=0]

    new_pc = new_pc[(new_pc[:,0]<50)*(new_pc[:,0]>-50)]
    new_pc = new_pc[(new_pc[:,1]<100)]
    new_pc = new_pc[(new_pc[:,2]<2)]

    simobj = Sceneset(new_pc)

    [rho, theta, loss, speed, angles] = simobj.specularpoints(radarobj)
    print(f"Number of points = {rho.shape[0]}")
    return rho, theta, loss, speed, angles

def run_lidar(sim_config, sem_lidar_frame):

    #restructed lidar.py code

    # lidar_path = f'{base_folder}/{sim_config["CARLA_SHENRON_SEM_LIDAR"]}'
    # # lidar_velocity_path = f'{base_folder}/{sim_config["LIDAR_PATH_POINT_VELOCITY"]}/'
    # out_path = f'{base_folder}/{sim_config["RADAR_PATH_SIMULATED"]}'

    # if not os.path.exists(out_path):
    #     os.makedirs(out_path)

    # shutil.copyfile('ConfigureRadar.py',f'{base_folder}/radar_params.py')

    # lidar_files = os.listdir(lidar_path)
    # lidar_velocity_files = os.listdir(lidar_velocity_path)
    # lidar_files.sort()
    # lidar_velocity_files.sort()

    # print(lidar_files)
    
    #Lidar specific settings
    radarobj = radar(sim_config["RADAR_TYPE"])
    # radarobj.chirps = 128
    radarobj.center = np.array([0.0, 0.0])  # center of radar
    radarobj.elv = np.array([0.0])

    # setting the sem lidar inversion angle here
    veh_angle = sim_config['INVERT_ANGLE']
    
    # all_speeds = []

    # temp_angles = []
    # temp_rho = []
    # for file_no, file in enumerate(lidar_files):
        
    #     start = time.time()
    #     if file.endswith('.npy'):  # .pcd
    #         print(file)
            
    #         lidar_file_path = os.path.join(f"{lidar_path}/", file)
    #         load_pc = np.load(lidar_file_path)

            # load_velocity = np.load(f'{lidar_velocity_path}/{file}')

            # test = map_material(test)
    cosines = sem_lidar_frame[:, 3]
    load_pc = sem_lidar_frame
    load_pc = map_carla_semantic_lidar_latest(load_pc)
    test = new_map_material(load_pc)
    
    points = np.zeros((np.shape(test)[0], 7))
    # points[:, [0, 1, 2]] = test[:, [0, 2, 1]]
    points[:, [0, 1, 2]] = test[:, [1, 0, 2]]

    """
    points mapping
    +ve ind 0 == right
    +ve ind 1 == +ve depth
    +ve ind 2 == +ve height
    """
    # add the velocity channel here to the lidar points on the channel number 3 most probably
    # points[:, 3] = load_velocity

    points[:, 5] = test[:, 3]
    points[:, 6] = cosines
    ### if jason carla lidar
    # points = np.zeros((np.shape(test)[0], 6))
    # points[:, [0, 1, 2]] = load_pc[:, [0, 1, 2]]
    # points[:, 5] = 4
    ##########

    # if USE_DIGITAL_TWIN:
    #     gt_label = gt[file_no,:]
    #     points, veh_speeds = create_digital_twin(points, gt_label) ## This also claculates and outputs speed

    #     all_speeds.append(veh_speeds)

    # if sim_config["RADAR_MOVING"]:
    #     # when the radar is moving, we add a negative doppler from all the points
    #     if INDOOR:
    #         curr_radar_speed = radar_speeds[file_no,:]

    #         cos_theta = (points[:,1]/np.linalg.norm(points[:,:2],axis=1))
    #         radial_speed = -np.linalg.norm(curr_radar_speed)*cos_theta

    #         points[:,3] += radial_speed
    #         points[:,5] = 4 ## harcoded 

    
    Crop_rho, Crop_theta, Crop_loss, Crop_speed, Crop_angles = Cropped_forRadar(points, np.array([0, 0, 0]), veh_angle, radarobj)

    """ DEBUG CODE
    spec_angle_thresh = 2*np.pi/180#*(1/rho)

    print(f"Number of points < 2deg = {np.sum(abs(Crop_angles)<spec_angle_thresh)}")
    temp_angles.append(np.sum(abs(Crop_angles)<spec_angle_thresh))
    temp_rho.append(Crop_rho.shape[0])
    continue
    """
    

    if sim_config["RAY_TRACING"]:
        rt_rho, rt_theta = ray_trace(points)

        rt_loss = np.mean(Crop_loss)*np.ones_like(rt_rho)
        rt_speed = np.zeros_like(rt_rho)
        Crop_rho = np.append(Crop_rho, rt_rho)
        Crop_theta = np.append(Crop_theta, rt_theta)
        Crop_loss = np.append(Crop_loss, rt_loss)
        Crop_speed = np.append(Crop_speed, rt_speed)

    adc_data = heatmap_gen(Crop_rho, Crop_theta, Crop_loss, Crop_speed, radarobj, 1, 0)
    return adc_data
    # check_save_path(out_path)
    # np.save(f'{out_path}/{file[:-4]}', adc_data)
    # diction = {"adc_data": adc_data}
    # sio.savemat(f"{out_path}/{file[:-4]}.mat", diction)
    # sio.savemat(f"test_pc.mat", diction)
    # print(f'Time: {time.time()-start}')
    # np.save("all_speeds_no_micro.npy",np.array(all_speeds))
    """ DEBUG CODE
    fig, ax = plt.subplots(1,2)
    ax[0].plot(temp_angles)
    ax[1].plot(temp_rho)
    
    plt.plot(temp_rho)
    plt.show()
    pdb.set_trace()
    """ 

if __name__ == '__main__':

    points = np.zeros((100,6))

    points[:,5] = 4
    
    points[:,0] = 1
    points[:,1] = np.linspace(0,15,100)
    
    points[:,3] = -0.5*np.cos(np.arctan2(points[:,0],points[:,1]))
    radarobj = radar('radarbook')
    # radarobj.chirps = 128
    radarobj.center = np.array([0.0, 0.0])  # center of radar
    radarobj.elv = np.array([0.0])

    Crop_rho, Crop_theta, Crop_loss, Crop_speed = Cropped_forRadar(points, np.array([0, 0, 0]), 0, radarobj)
    Crop_loss = np.ones_like(Crop_loss)
    adc_data = heatmap_gen(Crop_rho, Crop_theta, Crop_loss, Crop_speed, radarobj, 1, 0)
    diction = {"adc_data": adc_data}
    sio.savemat(f"test_pc.mat", diction)
