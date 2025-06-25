import numpy as  np
import os
import pickle as pkl
from PIL import Image
import json
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import cv2
from tqdm import tqdm
import laspy
import glob
import pdb

import sys
sys.path.append('/radar-imaging-dataset/carla-radarimaging/team_code/')

from sim_radar_utils.radar_processor import RadarProcessor
from sim_radar_utils.utils_radar import *
from sim_radar_utils.transform_utils import *


def check_save_path(save_path):
    os.makedirs(save_path, exist_ok=True)

def create_video(image_path, save_path, video_num):
    images = sorted((glob.glob(f'{image_path}/*.png')))
    img_array = []
    for image in images:
        img = cv2.imread(image)
        height, width, layers = img.shape
        global size
        size = (width,height)
        img_array.append(img)
    print(len(img_array))
    out = cv2.VideoWriter(f'{save_path}/video{video_num}.mp4', cv2.VideoWriter_fourcc(*'DIVX'), 7, size)
    for i in range(len(img_array)):
        out.write(img_array[i])
    out.release()

def generate_video(eval_data_path, eval_name, route_name, video_save_directory, iteration_num = 0):
    complete_data_path = os.path.join(eval_data_path, eval_name, f'Iteration{iteration_num}', route_name)
    path_radar_sim = os.path.join(complete_data_path, 'radar_img_eval')
    path_lidar_sim = os.path.join(complete_data_path, 'lidar_eval')
    path_rgb_sim = os.path.join(complete_data_path, 'rgb_eval')

    video_num = int(route_name.split('_')[-6][5 : ])
    video_save_path = os.path.join(video_save_directory, eval_name, f'Video{video_num}')
    img_save_path = os.path.join(video_save_path, 'Images')
    
    check_save_path(video_save_path)
    check_save_path(img_save_path)
    
    files_sim = os.listdir(path_radar_sim)
    files_sim.sort()
    
    files_sim1 = os.listdir(path_lidar_sim)
    files_sim1.sort()
    
    files_sim2 = os.listdir(path_rgb_sim)
    files_sim2.sort()
    
    print(f"Generating Video: {video_num}")
    
    radarProcessor = RadarProcessor()
    x = int(np.ceil(np.log10(len(files_sim1))))
    for i in tqdm(range(len(files_sim1)), desc="Processing"):
        rgb = mpimg.imread(f'{path_rgb_sim}/{files_sim2[i]}')
        
        radar_img = np.load(f'{path_radar_sim}/{files_sim[i]}')
        radar_img = np.log(radar_img+1e-10)

        las = laspy.read(f'{path_lidar_sim}/{files_sim1[i]}')
        
        dataset = np.vstack((las.x, las.y, las.z)).transpose()
        
        fig, ax = plt.subplot_mosaic("AA;BC", figsize=(8,5))

        ax['A'].imshow(rgb, aspect='auto')
        ax['A'].set_title('Camera Front')
        ax['A'].set_xticks([])
        ax['A'].set_yticks([])
        
        ax['B'].scatter(dataset[:, 1], dataset[:, 0], s=0.1, c=dataset[:, 2])
        ax['B'].set_title('Semantic Lidar')
        ax['B'].set_xlim(-50, 50)
        ax['B'].set_ylim(-50, 50)
        ax['B'].set_xticks([])
        ax['B'].set_yticks([])
        
        ax['C'].imshow(radar_img, aspect='auto')
        ax['C'].set_title('Shenron Radar')
        ax['C'].set_xticks([])
        ax['C'].set_yticks([])
        
        plt.tight_layout()
        
        plt.savefig(f'{img_save_path}/{i:0{x}}.png')
        plt.close()

    create_video(img_save_path, video_save_path, video_num)
    
def main():
    eval_data_path = '/radar-imaging-dataset/carla-radarimaging/Evaluation_Results'
    eval_name = 'NEAT_Evaluations_fblr_redact_left_video'
    
    video_save_path = '/radar-imaging-dataset/carla-radarimaging/Evaluation_Results/Videos'
    
    route_name = os.listdir(os.path.join(eval_data_path, eval_name, "Iteration0"))
    route_name = [name for name in route_name if "route" in name]
    route_name.sort()
    
    for route in route_name[7:]:
        print("Generating Video for Route: ", route)
        generate_video(eval_data_path, eval_name, route, video_save_path, iteration_num = 0)
    
if __name__=="__main__":
    main()
    
# def main():
#     radarProcessor = RadarProcessor()
#     main_directory = "NEAT_Evaluations_new_fblr_video_v3_retry/"
#     video_num = 3 # Change this to vary video number
#     route_directory = "routes_eval_NEAT_route2_10_22_16_03_24" # Change this to select route
    
#     path_to_dir = '/radar-imaging-dataset/carla-radarimaging/Evaluation_Results/' + main_directory + '/Iteration0/' + route_directory + '/'
#     path_radar_sim = path_to_dir + 'radar_img_eval'
#     path_lidar_sim = path_to_dir + 'lidar_eval' #semantic lidar
#     path_rgb_sim = path_to_dir + 'rgb_eval' #rgb
    
#     video_directory = "NEAT_Evaluations_new_fblr_video_v3_retry"
#     video_save_path = '/radar-imaging-dataset/carla-radarimaging/Evaluation_Results/Videos/' + video_directory + '/Video' + str(video_num) + '/'
#     img_path = video_save_path + "/Images/"
    
#     check_save_path(video_save_path)
#     check_save_path(img_path)
    
#     files_sim = os.listdir(path_radar_sim)
#     files_sim.sort()

#     files_sim1 = os.listdir(path_lidar_sim)
#     files_sim1.sort()
    
#     files_sim2 = os.listdir(path_rgb_sim)
#     files_sim2.sort()
    
#     radarProcessor = RadarProcessor()

#     print(f"Generating Video: {video_num}")
#     x = int(np.ceil(np.log10(len(files_sim1))))
#     for i in tqdm(range(len(files_sim1)), desc="Processing"):
#         # radar_data_simulated processing
#         rgb = mpimg.imread(f'{path_rgb_sim}/{files_sim2[i]}')
#         # radar
#         radar_img = np.load(f'{path_radar_sim}/{files_sim[i]}')
#         radar_img = np.log(radar_img+1e-10)
#         '''
#         # radar back
#         radar_back = np.load(f'{path_sim_back}/{files_sim_back[i]}')
#         chirpLevelData_back = reformat_adc_shenron(radar_back)
#         rangeProfile_back = radarProcessor.cal_range_fft(chirpLevelData_back)
#         # dopplerProfile = radarProcessor.cal_doppler_fft(rangeProfile)
#         aoaProfile_back = radarProcessor.cal_angle_fft(rangeProfile_back)
#         range_angle_back = get_range_angle(aoaProfile_back)
#         cart_cord_back = polar_to_cart(range_angle_back, limit = 50)
#         # normalizing the image
#         mean_all_back = np.mean(cart_cord_back)
#         std_all_back = np.std(cart_cord_back)
#         normalized_back = (cart_cord_back-mean_all_back)/std_all_back
#         cart_cord_log_back = np.log(cart_cord_back+1e-10)
#         # concatenating front and back images
#         cart_cord_back = np.rot90(np.rot90(cart_cord_back))
#         radar_cat = np.concatenate((cart_cord, cart_cord_back), axis=0)
#         center_x, center_y = radar_cat.shape[1] // 2, radar_cat.shape[0] // 2
#         crop_size = 256
#         radar_cropped = radar_cat[center_y - crop_size // 2:center_y + crop_size // 2,
#                         center_x - crop_size // 2:center_x + crop_size // 2]
#         # pdb.set_trace()
#         # print(radar_cropped.shape)
#         radar_cropped_log = np.log(radar_cropped+1e-10)
#         '''
#         # for lidar
#         las = laspy.read(f'{path_lidar_sim}/{files_sim1[i]}')
#         # dataset = np.vstack((las.x, las.y, las.z, las.cosine, las.index, las.sem_tag)).transpose()
#         dataset = np.vstack((las.x, las.y, las.z)).transpose()
#         # radar1 = np.load(f'{path_sim1}/{files_sim1[i]}')
#         # chirpLevelData1 = reformat_adc_shenron(radar1)
#         # rangeProfile1 = radarProcessor.cal_range_fft(chirpLevelData1)
#         # # dopplerProfile = radarProcessor.cal_doppler_fft(rangeProfile)
#         # aoaProfile1 = radarProcessor.cal_angle_fft(rangeProfile1)
#         # range_angle1 = get_range_angle(aoaProfile1)
#         # cart_cord1 = polar_to_cart(range_angle1, limit = 50)
#         # cart_cord_log1 = np.log(cart_cord1+1e-10)
#         # lidar_data processing
#         # lidar = np.load(f"{path_sem_lidar}/{files_sem_lidar[i]}")
#         # las = laspy.read(f"{path_sem_lidar}/{files_sem_lidar[i]}")
#         # Grab a numpy dataset of our clustering dimensions:
#         # dataset = np.vstack((las.x, las.y, las.z, las.cosine, las.index, las.sem_tag)).transpose()
#         # load_pc = np.load(lidar_file_path)
#         # lidar = dataset
#         # print(lidar.shape)
#         # # pdb.set_trace()
#         # # radar_data_sim_manual processing
#         # radar_man = np.load(f'{path_sim_man}/{files_sim_manual[i]}')
#         # chirpLevelData = reformat_adc_shenron(radar_man)
#         # rangeProfile = radarProcessor.cal_range_fft(chirpLevelData)
#         # # dopplerProfile = radarProcessor.cal_doppler_fft(rangeProfile)
#         # aoaProfile = radarProcessor.cal_angle_fft(rangeProfile)
#         # range_angle = get_range_angle(aoaProfile)
#         # cart_cord_man = polar_to_cart(range_angle, limit = 50)
#         # print(cart_cord_man.shape)
#         fig, ax = plt.subplot_mosaic("AA;BC", figsize=(8,5))
#         # fig, ax1 = plt.subplots(1, 1, figsize=(5,5))
#         ax['A'].imshow(rgb, aspect='auto')
#         ax['A'].set_title('Camera Front')
#         ax['B'].scatter(dataset[:, 1], dataset[:, 0], s=0.1, c=dataset[:, 2])
#         ax['B'].set_title('Semantic Lidar')
#         # ax['C'].imshow(radar, aspect='auto')
#         ax['C'].imshow(radar_img, aspect='auto')
#         '''
#         ax['C'].imshow(radar_cropped_log, aspect='auto')
#         '''
#         ax['C'].set_title('Shenron Radar')
#         # ax2.imshow(cart_cord, aspect='auto')
#         # ax2.set_title('radar data simulated manually')
#         # # plt.show()
#         # ax2.scatter(lidar[:, 1], lidar[:, 0], s = 0.1, c = lidar[:, 5])
#         # # ax2.scatter(lidar[:, 0], lidar[:, 1], s = 0.1, c = lidar[:, 5])
#         ax['B'].set_xlim(-50, 50)
#         ax['B'].set_ylim(-50, 50)
#         # ax2.set_title('lidar data')
#         plt.tight_layout()
#         # plt.show()
#         plt.savefig(f'{img_path}{i:0{x}}.png')
#         plt.close()
#         # break
    
    
#     files_img = os.listdir(img_path)
#     files_img.sort()

#     x = int(np.ceil(np.log10(len(files_img))))
    
#     for file in os.listdir(img_path):
#         os.rename(img_path + file, img_path + f"{int(file.split('.')[0]):0{x}}.png")

#     create_video(img_path, video_save_path, video_num)