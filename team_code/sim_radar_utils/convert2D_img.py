import numpy as  np
import os
import pickle as pkl
from PIL import Image
import json
import yaml
# import ffmpeg
import matplotlib.pyplot as plt
# import cv2
import sys
import glob

# sys.path.append('/radar-imaging-dataset/mmfn_project/mmfn_scripts/team_code/mmfn_utils/sim_radar_utils/')
# sys.path.append('/radar-imaging-dataset/mmfn_project/mmfn_scripts/team_code/e2e_agent_sem_lidar2shenron_package/')

from e2e_agent_sem_lidar2shenron_package.lidar import run_lidar

from sim_radar_utils.radar_processor import RadarProcessor
from sim_radar_utils.utils_radar import *
from sim_radar_utils.transform_utils import *

def convert2D_img_func(sim_radar, limit = 75):
    '''
    converts the 3D radar raw data to 2d range-angle image of dimension 256X256
    '''
    radarProcessor = RadarProcessor()
    # radar = sim_radar[0]
    radar = sim_radar
    
    chirpLevelData = reformat_adc_shenron(radar)
    rangeProfile = radarProcessor.cal_range_fft(chirpLevelData)
    # dopplerProfile = radarProcessor.cal_doppler_fft(rangeProfile)
    aoaProfile = radarProcessor.cal_angle_fft(rangeProfile)
    range_angle = get_range_angle(aoaProfile)
    cart_cord = polar_to_cart(range_angle, limit = limit)
    # print(cart_cord_man.shape)
    # return [cart_cord]
    return cart_cord

def convert_sem_lidar_2D_img_func(sim_radar, invert_angle, limit = 75):
    '''
    converts the semantic_lidar data to the range angle 2D image of dimension 256X256
    '''
    
    # convert semantic lidar to raw 3d radar data
    
    # with open('/radar-imaging-dataset/mmfn_project/mmfn_scripts/team_code/e2e_agent_sem_lidar2shenron_package/simulator_configs.yaml', 'r') as f:
    #     sim_config = yaml.safe_load(f)
    
    # with open(glob.glob('../carla_garage_radar/team_code/e2e_agent_sem_lidar2shenron_package/simulator_configs.yaml'), 'r') as f:
    #     sim_config = yaml.safe_load(f)
    
    with open('/radar-imaging-dataset/carla_garage_radar/team_code/e2e_agent_sem_lidar2shenron_package/simulator_configs.yaml', 'r') as f:
        sim_config = yaml.safe_load(f)
    
    sim_config['INVERT_ANGLE'] = invert_angle
    
    radar = run_lidar(sim_config, sim_radar)
       
    radarProcessor = RadarProcessor()
    # radar = sim_radar[0]
    # radar = sim_radar
    
    chirpLevelData = reformat_adc_shenron(radar)
    rangeProfile = radarProcessor.cal_range_fft(chirpLevelData)
    # dopplerProfile = radarProcessor.cal_doppler_fft(rangeProfile)
    aoaProfile = radarProcessor.cal_angle_fft(rangeProfile)
    range_angle = get_range_angle(aoaProfile)
    cart_cord = polar_to_cart(range_angle, limit = limit)
    # print(cart_cord_man.shape)
    # return [cart_cord]
    return cart_cord
    
def main():
    radarProcessor = RadarProcessor()

    # with open('/mnt/intA-ssdr1-4tb/satyam53/mmfn/data1/pro_train_fnss_0702/0.pkl', 'rb') as f:
    #     pkl_data = pkl.load(f)
    
    # print(type(pkl_data))
    # print(pkl_data.keys())
    
    # sim_radar = pkl_data['sim_radar']
    # # sim_radar = pkl_data['vectormaps']   
    
    # print(type(sim_radar[0]))
    # print(len(sim_radar))
    # print(sim_radar[0])
    # radar_man = sim_radar[0]
    
    
    # # radar_man = np.load(f'{path_sim_man}/{files_sim_manual[i]}')
    # chirpLevelData = reformat_adc_shenron(radar_man)
    # rangeProfile = radarProcessor.cal_range_fft(chirpLevelData)
    # # dopplerProfile = radarProcessor.cal_doppler_fft(rangeProfile)
    # aoaProfile = radarProcessor.cal_angle_fft(rangeProfile)
    # range_angle = get_range_angle(aoaProfile)
    # cart_cord_man = polar_to_cart(range_angle, limit = 50)
    # print(cart_cord_man.shape)
    
    # fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15,5))
    # # ax1.imshow(cart_cord, aspect='auto')
    # # ax1.set_title('radar data simulated using shenron')

    # ax2.imshow(cart_cord_man, aspect='auto')
    # ax2.set_title('radar data simulated manually')
    # # plt.show()
    # # ax3.scatter(lidar[:, 0], lidar[:, 1], s = 0.1, c = lidar[:, 5])
    # # ax3.set_title('lidar data')
    # plt.tight_layout()
    # plt.show()
    # # plt.savefig(f'{save_path}/{i}.png')
    # plt.close()
    
if __name__=="__main__":
    main()