import numpy as np
import open3d as o3d
import os
import scipy.io as sio
import sys
sys.path.append("../")
# from path_config import *
import pdb

def create_digital_twin(lidar_points, gt_labels):

    """
    Augments the input point cloud using the ground truth labels.
    Could densify or replace dynamic objects with CAD
    Input:
        lidar_points: Nx6
        gt_labels: GT boxes for the lidar frame

    Output:
        digital_twin: N'x6 Augmented point cloud
    """
    cad_id = 1
    # pdb.set_trace()
    # car_data = sio.loadmat(path_config["path_cad"] + f"car_cart{cad_id}.mat")
    # car_cart = np.array(car_data['car_cart'])
    
    car_data = sio.loadmat(path_config["path_cad"] + f"chassis_camry.mat")
    car_cart = np.array(car_data['chassis_loc'])

    wheel_data = sio.loadmat(path_config["path_cad"] + f"wheel_camry.mat")
    wheel_cart = np.array(wheel_data['wheel_loc'])

    speed_multiplier = sio.loadmat(path_config["path_cad"] + f"speed_multiplier.mat")
    speed_multiplier = np.array(speed_multiplier['mult_all'])

    speed_multiplier = np.concatenate((np.ones(car_cart.shape[0],), np.squeeze(speed_multiplier)), axis=0)
    car_cart = np.concatenate((car_cart, wheel_cart), axis=0)


    veh_angle = gt_labels[-1]
    rotMatrix = np.array([[np.cos(np.deg2rad(veh_angle)),
                           np.sin(np.deg2rad(veh_angle))]
                             , [- np.sin(np.deg2rad(veh_angle)),
                                np.cos(np.deg2rad(veh_angle))]])
            

    car_cart[:, 0:2] = np.matmul(car_cart[:, 0:2], np.array([[0, -1], [1, 0]]))
    gt_dims = gt_labels[3:6]
    # dims_mat = sio.loadmat(path_config["path_cad"] + "dimensions.mat")
    # dims_mat = dims_mat['dims']
    # cad_dims = np.squeeze(dims_mat[dims_mat[:,3]==cad_id,:3]).tolist()

    cad_dims = np.array([4.805,1.795,1.495])

    
    scale = gt_dims/cad_dims

    new_car_cart = np.matmul(car_cart[:, 0:2], rotMatrix)

    veh_x_new = gt_labels[1] + new_car_cart[:, 0]*scale[0]
    veh_y_new = -gt_labels[0] + new_car_cart[:, 1]*scale[1]
    veh_z_new = -1 + car_cart[:, 2]*scale[2]  # get the ground height
    
    ###calculate radial_speed
       
    speed = np.ones((veh_x_new.shape[0],2))*gt_labels[9:11]
    # speed[points[:,5]==0.1,:] = gt_labels[9:11]
    # x = -points[:,1]
    # y = points[:,0]
    x = -veh_y_new
    y = veh_x_new

    xy = np.hstack((x[:,None],y[:,None]))
    rad_speed = np.sum((speed*xy)/np.linalg.norm(xy,axis=1)[:,None],1)
    
    veh_speed = rad_speed#*speed_multiplier

    # veh_speed = np.zeros(np.size(new_car_cart[:, 0]))
    veh_angle = np.zeros(np.size(new_car_cart[:, 0]))
    veh_material = 0.1 * np.ones(np.size(new_car_cart[:, 0])) ## Car material
    


    car_pc = np.vstack((veh_x_new, veh_y_new, veh_z_new, veh_speed, veh_angle, veh_material)).T


    


    digital_twin = np.vstack((lidar_points,car_pc))

    return digital_twin, veh_speed

def map_material(test):
    # test = test[test[:, 1] > 0, :]  # filter to obtain calibration points
    # test = test[test[:, 1] < 10, :]
    # test[test[:, 3] != 255, 3] = 5

    
    # amplify = 1
    # after metal tuning - 0.01,0.1,0.001
    conc = 0.03  # .04  # 0.3  # 2  # best  # calculated  # lucky
    steel = 0.1  # 0.99  # 200
    tree = 0.001  # 0.0001  # 0.01  # 0.02
    human = 0.1 
    test[test[:, 3] == 255, 3] = 0  # for unlabeled assign 0
    test[test[:, 3] == 0, 3] = 0  # for road->concrete, 0 to suppress the ground rings that show up as objects
    test[test[:, 3] == 1, 3] = conc  # for sidewalk->concrete
    test[test[:, 3] == 2, 3] = conc  # for building->concrete
    test[test[:, 3] == 3, 3] = conc  # for wall->concrete
    test[test[:, 3] == 4, 3] = steel  # for fence->metal
    test[test[:, 3] == 5, 3] = steel  # for pole->metal
    test[test[:, 3] == 6, 3] = steel  # for traffic light->metal
    test[test[:, 3] == 7, 3] = steel  # for traffic sign->metal
    test[test[:, 3] == 8, 3] = tree  # for vegetation->trees  # just for wall data
    test[test[:, 3] == 9, 3] = tree  # for terrain->trees  # just for wall data
    test[test[:, 3] == 10, 3] = 0  # for sky->0
    test[test[:, 3] == 11, 3] = human  # for person->metal (reflects significant energy, approx)
    test[test[:, 3] == 12, 3] = steel  # for rider->metal
    test[test[:, 3] == 13, 3] = steel  # for car->metal
    test[test[:, 3] == 14, 3] = steel  # for truck->metal
    test[test[:, 3] == 15, 3] = steel  # for bus->metal
    test[test[:, 3] == 16, 3] = steel  # for train->metal
    test[test[:, 3] == 17, 3] = steel  # for motorbike->metal
    test[test[:, 3] == 18, 3] = steel  # for bicycle->metal

    return test

def new_map_material(test):
    # map material to index 
    #0 -> unlabeled
    #1 -> wood
    #2 -> concrete
    #3 -> human
    #4 -> metal
    
    unlabel = 0
    wood = 1
    conc = 2
    human = 3 
    metal = 4

    unlabel_roughness = 0
    wood_roughness = 0.0017 #1.7mm
    conc_roughness = 0.0017 #1.7mm
    human_roughness = 0.0001 # 100um
    metal_roughness = 0.0001 # 100um 
    
    roughness = np.array([unlabel_roughness,wood_roughness,conc_roughness,human_roughness,metal_roughness])

    unlabel_perm = 1
    wood_perm = 2 
    conc_perm = 5.24 
    human_perm = 15 
    metal_perm = 100000
    
    permittivity = np.array([unlabel_perm,wood_perm,conc_perm,human_perm,metal_perm])


    test[test[:, 3] == 255, 3] = unlabel  # for unlabeled assign 0
    test[test[:, 3] == 0, 3] = unlabel  # for road->unlabeled, to suppress the ground rings that show up as objects
    test[test[:, 3] == 1, 3] = conc  # for sidewalk->concrete
    test[test[:, 3] == 2, 3] = conc  # for building->concrete
    test[test[:, 3] == 3, 3] = conc  # for wall->concrete
    test[test[:, 3] == 4, 3] = metal  # for fence->metal
    test[test[:, 3] == 5, 3] = metal  # for pole->metal
    test[test[:, 3] == 6, 3] = metal  # for traffic light->metal
    test[test[:, 3] == 7, 3] = metal  # for traffic sign->metal
    test[test[:, 3] == 8, 3] = wood  # for vegetation->trees  # just for wall data
    test[test[:, 3] == 9, 3] = wood  # for terrain->trees  # just for wall data
    test[test[:, 3] == 10, 3] = unlabel  # for sky->0
    test[test[:, 3] == 11, 3] = human  # for person->metal (reflects significant energy, approx)
    test[test[:, 3] == 12, 3] = metal  # for rider->metal
    test[test[:, 3] == 13, 3] = metal  # for car->metal
    test[test[:, 3] == 14, 3] = metal  # for truck->metal
    test[test[:, 3] == 15, 3] = metal  # for bus->metal
    test[test[:, 3] == 16, 3] = metal  # for train->metal
    test[test[:, 3] == 17, 3] = metal  # for motorbike->metal
    test[test[:, 3] == 18, 3] = metal  # for bicycle->metal
    
    return test

if __name__ == '__main__':

    gt = sio.loadmat("../gt_11_08_car_round2.mat")
    gt = gt["a"]
    

    # print(gt_label)
    exp_date = "11_08"
    lidar_path = f"Y:/media/Kshitiz/RADARIMAGING/radar_simulator_data/{exp_date}_data/{exp_date}_lidar_appended"
    lidar_folder = f"test_11_08_car_round2"
    # file = "0001.npy"
    files = np.array(os.listdir(f"{lidar_path}/{lidar_folder}/"))

    select_array = np.arange(50,51)
    for file_idx, file in enumerate(files[select_array]):
        lidar_file_path = os.path.join(f"{lidar_path}/{lidar_folder}/", file)
        
        test = np.load(lidar_file_path)
        test = map_material(test)
            
        points = np.zeros((np.shape(test)[0], 6))
        
        points[:, [0, 1, 2]] = test[:, [0, 2, 1]]
        
        points[:, 5] = test[:, 3]

        gt_label = gt[select_array[file_idx],:]
        points = create_digital_twin(points, gt_label)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[:, 0:3])
        o3d.visualization.draw_geometries([pcd])

    