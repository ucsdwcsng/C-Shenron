from re import A
import open3d as o3d
import os
import numpy as np
import math
import matplotlib.pyplot as plt
import pdb
from e2e_agent_sem_lidar2shenron_package.ConfigureRadar import radar
import sys

def cart2sph(x, y, z):
    hxy = np.hypot(x, y)
    r = np.hypot(hxy, z)
    el = np.arctan2(z, hxy)
    az = np.arctan2(y, x)
    return az, el, r

def sph2cart( az, el, r):
    rcos_theta = r * np.cos(el)
    x = rcos_theta * np.cos(az)
    y = rcos_theta * np.sin(az)
    z = r * np.sin(el)
    return x, y, z

class Sceneset():

    """
    Class with all the functions to modify OSM scenario and thus generate the points for radar heatmap
    """
    def __init__(self, pc):
        self.points_3D = pc
        self.rad_scene = self.points_3D
        pass

    def removeocclusion(self, radar):

        row = np.where(self.points_3D.any(axis=1))[0]
        self.points_3D = self.points_3D[row, :]

        # to reduce volume to 100x150x25, else voxel filter too big
        rowx = np.where((self.points_3D[:, 0] < 50) & (self.points_3D[:, 0] > -50))  # x limits
        self.points_3D = self.points_3D[rowx, :].squeeze(0)
        rowy = np.where((self.points_3D[:, 1] < 150) & (self.points_3D[:, 1] > 0))  # y limits
        self.points_3D = self.points_3D[rowy,:].squeeze(0)
        rowz = np.where((self.points_3D[:, 2] < 5) & (self.points_3D[:, 2] > -2)) #z limits
        self.points_3D = self.points_3D[rowz,:].squeeze(0)
        rowx = []
        rowy = []
        rowz = []

        self.rad_scene = self.points_3D
        sph_v = np.zeros((self.points_3D.shape[0], 3))
        sph_v[:, 0], sph_v[:, 1], sph_v[:, 2] = cart2sph(self.points_3D[:, 0]-radar.center[0], self.points_3D[:, 1]-radar.center[1], self.points_3D[:, 2]-radar.elv)
        sphlim = np.array([np.min(sph_v, axis=0), np.max(sph_v, axis=0)]) # limits in all three dimensions in the spherical coordinates


        pcd2 = o3d.geometry.PointCloud()
        pcd2.points = o3d.utility.Vector3dVector(self.rad_scene[:, 0:3])
        # o3d.visualization.draw_geometries([pcd2])
        

        # convert the point cloud into a grid of 0s and 1s
        # define voxel size of the grid
        phi_res = radar.voxel_phi / 180 * math.pi
        theta_res = radar.voxel_theta / 180 * math.pi
        rho_res = radar.voxel_rho

        sph_m_phi = np.arange(sphlim[0, 0], sphlim[1, 0] + phi_res,phi_res) + phi_res
        sph_m_theta = np.arange( sphlim[0, 1], sphlim[1, 1] + theta_res, theta_res) + theta_res
        sph_m_rho = np.arange(sphlim[0, 2], sphlim[1, 2] + rho_res, rho_res) + rho_res
        sph_m_size = [len(sph_m_phi), len(sph_m_theta), len(sph_m_rho)]
        sph_m = np.zeros(sph_m_size)
        vel_m = np.zeros(sph_m_size)
        mtl_m = np.zeros(sph_m_size)

        phi_m_idx = np.round_((sph_v[:, 0] - sphlim[0, 0]) / phi_res)  # +1
        theta_m_idx = np.round_((sph_v[:, 1] - sphlim[0, 1]) / theta_res)  # +1
        rho_m_idx = np.round_((sph_v[:, 2] - sphlim[0, 2]) / rho_res)  # +1
        for k_pt in range(0, len(self.points_3D)):
            sph_m[int(phi_m_idx[k_pt]), int(theta_m_idx[k_pt]), int(rho_m_idx[k_pt])] = 1
            vel_m[int(phi_m_idx[k_pt]), int(theta_m_idx[k_pt]), int(rho_m_idx[k_pt])] = self.points_3D[k_pt, 3]
            mtl_m[int(phi_m_idx[k_pt]), int(theta_m_idx[k_pt]), int(rho_m_idx[k_pt])] = self.points_3D[k_pt, 4]

        visible_sph_m = np.zeros(sph_m.shape)
        visible_vel_m = np.zeros(vel_m.shape)
        visible_mtl_m = np.zeros(mtl_m.shape)
        for kphi in range(0, sph_m_size[0]):
            for ktheta in range(0, sph_m_size[1]):
                if sph_m[kphi, ktheta, :].any() > 0:
                    krho = np.where(sph_m[kphi, ktheta, :] > 0)[0][0]
                    visible_sph_m[kphi, ktheta, krho] = sph_m[kphi, ktheta, krho]
                    visible_vel_m[kphi, ktheta, krho] = vel_m[kphi, ktheta, krho]
                    visible_mtl_m[kphi, ktheta, krho] = mtl_m[kphi, ktheta, krho]

        visible_sph_m_idx = np.where(visible_sph_m)
        # sph_v_idx = []
        sph_v_idx = np.array([visible_sph_m_idx[0], visible_sph_m_idx[1], visible_sph_m_idx[2]]).T  # np.unravel_index( visible_sph_m_idx, sph_m_size)
        visible_vel_v = visible_vel_m[visible_sph_m_idx]
        visible_mtl_v = visible_mtl_m[visible_sph_m_idx]
        visible_sph_v = np.array([sph_m_phi[sph_v_idx[:, 0]], sph_m_theta[sph_v_idx[:, 1]], sph_m_rho[sph_v_idx[:, 2]]]).T

        visible_cart_v = np.zeros(visible_sph_v.shape)
        [visible_cart_v[:, 0], visible_cart_v[:, 1], visible_cart_v[:, 2]] = sph2cart(visible_sph_v[:, 0], visible_sph_v[:, 1], visible_sph_v[:, 2])
        self.rad_scene = np.vstack((visible_cart_v[:, :].T, visible_vel_v, visible_mtl_v)).T+np.hstack((radar.center, radar.elv, np.array([0, 0])))
        

        # pcd2 = o3d.geometry.PointCloud()
        # pcd2.points = o3d.utility.Vector3dVector(self.rad_scene[:, 0:3])
        
        # o3d.visualization.draw_geometries([pcd2])
        

        # ground_refl = np.vstack((visible_cart_v[:, :].T, visible_vel_v, visible_mtl_v)).T+np.array(radar.center + [1] + [0, 0])
        # self.rad_scene = np.append(self.rad_scene, ground_refl*np.array([1, 1, -1, 1, 1]), axis=0)

    def removeocclusion_hiddenpoint(self, radar):
        row = np.where(self.points_3D.any(axis=1))[0]
        self.points_3D = self.points_3D[row, :]

        # to reduce volume to 100x150x25, else voxel filter too big
        rowx = np.where((self.points_3D[:, 0] < 50) & (self.points_3D[:, 0] > -50))  # x limits
        self.points_3D = self.points_3D[rowx, :].squeeze(0)
        rowy = np.where((self.points_3D[:, 1] < 150) & (self.points_3D[:, 1] > 0))  # y limits
        self.points_3D = self.points_3D[rowy,:].squeeze(0)
        rowz = np.where((self.points_3D[:, 2] < 5) & (self.points_3D[:, 2] > -2)) #z limits
        self.points_3D = self.points_3D[rowz,:].squeeze(0)
        rowx = []
        rowy = []
        rowz = []

        self.rad_scene = self.points_3D
        vel = self.points_3D[:,3]
        mtl = self.points_3D[:,4]

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.rad_scene[:, 0:3])
        # o3d.visualization.draw_geometries([pcd])


        diameter = np.linalg.norm(
            np.asarray(pcd.get_max_bound()) - np.asarray(pcd.get_min_bound()))

        print("Define parameters used for hidden_point_removal",diameter)
        camera = [radar.center[0], radar.center[1], radar.elv]
        radius = 10000
        # pdb.set_trace()
        print("Get all points that are visible from given view point")
        try:
            _, pt_map = pcd.hidden_point_removal(camera, radius)
        except Exception as e:
            pt_map = np.arange(self.points_3D.shape[0])
            print(e)

        
        self.rad_scene = np.hstack((np.array(pcd.points)[pt_map],vel[pt_map,None],mtl[pt_map,None]))
        
        print(f"Number of points in point cloud = {self.rad_scene.shape[0]}")
        vel = self.rad_scene[:,3]
        mtl = self.rad_scene[:,4]
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.rad_scene[:, 0:3])
        pcd_tree = o3d.geometry.KDTreeFlann(pcd)
        downpcd = pcd.voxel_down_sample(voxel_size=0.05)
        valid_points= np.zeros((1,5))
        for point in range(0,len(np.asarray(downpcd.points))):
            [_, idx,_] = pcd_tree.search_knn_vector_3d(downpcd.points[point], 1)
            valid_point = np.concatenate((np.array(pcd.points)[idx].T,vel[idx,None],mtl[idx,None]),axis=0).flatten()
            valid_points = np.concatenate((valid_points,[valid_point]),axis=0)
        valid_points=np.delete(valid_points,0,axis=0)    
        self.rad_scene = valid_points
        print(f"Number of points after downsampling = {self.rad_scene.shape[0]}")

        # print(f"Number of points in point cloud = {self.rad_scene.shape[0]}")
        # # downsample_ind = np.random.choice(self.rad_scene.shape[0], round(0.25*self.rad_scene.shape[0]))
        # try:
        #     downsample_ind = np.random.choice(self.rad_scene.shape[0], 4000) ## need to fix this by breaking the number of points in heatmapgen
        #     self.rad_scene = self.rad_scene[downsample_ind,:]
        # except Exception as e:
        #     pass


        
        # print(f"Number of points after downsampling = {self.rad_scene.shape[0]}")

        # pcd2 = o3d.geometry.PointCloud()
        # pcd2.points = o3d.utility.Vector3dVector(self.rad_scene[:, 0:3])
        # o3d.visualization.draw_geometries([pcd2])
        # sys.exit()

    def specularpoints(self, radar):

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.rad_scene[:, 0:3])

        pcd.estimate_normals( search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=10, max_nn=10))
        # o3d.visualization.draw_geometries([pcd])

        # plt.figure()
        # pcd = o3d.geometry.PointCloud()
        # pcd.points = o3d.utility.Vector3dVector(self.rad_scene)
        # o3d.visualization.draw_geometries([pcd])
        # plt.title('Estimated Normals of Point Cloud')

        x = np.array(pcd.points)[:, 0]
        y = np.array(pcd.points)[:, 1]
        z = np.array(pcd.points)[:, 2]
        u = np.array(pcd.normals)[:, 0]
        v = np.array(pcd.normals)[:, 1]
        w = np.array(pcd.normals)[:, 2]

        # fig = plt.figure()
        # ax = fig.gca(projection='3d')
        # ax.quiver(x, y, z, u, v, w)
        # plt.show()
        
        angles = np.arccos(np.sum(np.array(pcd.normals)[:, 0: 3]*((np.array([x, y, z])-np.hstack((radar.center, radar.elv)).reshape((3, 1))).T /np.linalg.norm(np.array([x, y, z])-np.hstack((radar.center, radar.elv)).reshape((3, 1)), axis=0)[:, np.newaxis]), axis=1))
        angles_carla = self.rad_scene[:, 5]
        spec_mask = self.rad_scene

        rho = np.linalg.norm(spec_mask[:, 0:3] - np.hstack((radar.center, radar.elv)), axis=1)
        theta = math.pi / 2 - np.arctan(((spec_mask[:, 0] - radar.center[0]) / (spec_mask[:, 1] - radar.center[1])))
        elev_angle = np.arccos((spec_mask[:, 2] - radar.elv)/rho)
        # loss = np.zeros((len(theta),), dtype=float)
        # for i in range(len(theta)):
        #     if self.rad_scene[i, 4] == 0.1:
        #         loss[i] = (0.02 * (((1 + np.abs(np.cos(angles[i]))) / 2) ** 0.01) + 0.15*np.abs(np.cos(angles[i]))**300) * self.rad_scene[i, 4] * rho[i] * rho[i] * 3282 * math.radians(radar.voxel_phi) * math.radians(radar.voxel_theta)  # + 0.1*np.abs(np.cos(angles))**300
        #     else:  # after metal without concrete 0.15,300 # after concrete 0.6,600 (just for the else loss)
        #         loss[i] = (0.02 * (((1 + np.abs(np.cos(angles[i]))) / 2) ** 0.01) + 0.6*np.abs(np.cos(angles[i]))**600) * self.rad_scene[i, 4] * rho[i] * rho[i] * 3282 * math.radians(radar.voxel_phi) * math.radians(radar.voxel_theta)  # radar.voxel_phi * radar.voxel_theta
        speed = self.rad_scene[:, 3]  # 0.05&5 , 0.1&300
        


        # file_path = os.path.join('F:/28_07_data/28_07_lidar_data/test_28_7_run11/mtlb-pcdFromBag', '0001.pcd')
        # pc = o3d.io.read_point_cloud(file_path)
        # pc.points = o3d.utility.Vector3dVector(self.rad_scene[:, 0:3])
        # o3d.visualization.draw_geometries([pc])
        loss_att,_,_ = get_loss_3(self.rad_scene, rho, elev_angle , angles_carla, radar, use_spec = False, use_diffused = True, no_material=False)

        return rho, theta, loss_att, speed, angles


def get_loss(points, rho, angles, radar, use_spec = True, use_diffused = True, no_material = False):
    
    # loss = np.zeros((len(theta),), dtype=float)
    
    if no_material:
        points[:,4] = 1

    constant = points[:, 4] * rho * rho * 3282 * math.radians(radar.voxel_phi) * math.radians(radar.voxel_theta) # N-points length
    
    diff_scatter = 0.02 * (((1 + np.abs(np.cos(angles))) / 2) ** 0.01) # N-points length

    spec_gain = 0.6 * np.ones(len(angles),)
    spec_exp = 600 * np.ones(len(angles),)

    #for metal
    spec_gain[points[:, 4] == 0.1] = 0.15
    spec_exp[points[:, 4] == 0.1] = 300

    specular = spec_gain*np.abs(np.cos(angles))**spec_exp

    loss = (use_diffused * diff_scatter + use_spec * specular) * constant

    return loss, diff_scatter*constant, specular*constant


def get_loss_2(points, rho, angles, radar, use_spec = True, use_diffused = True, no_material = False):
    '''
    Note: angles should be used for power calculation as it is in reflecting surface coordinate system while theta is in radar coordinate system

    Implemement power conservation here:
    P_inc = P_return + P_trans
    P_inc = P_inc * T^2 + P_inc * (1 - T^2)
    P_inc = P_inc * T^2 (R^2 + S^2) + P_trans

    Assume P_trans=0 or T=1. This can be more accurately calculated from fresnel equations
    
    P_inc = (P_inc * R^2) + (P_inc * S^2)
    P_inc = P_spec + P_scat
    '''
    
    if no_material:
        points[:,4] = 1


    ##Assuming rho and theta are calculated correctly
    P_spec_inc = np.zeros((len(angles),), dtype=float)
    P_scat_inc = np.zeros((len(angles),), dtype=float)

    K_sq = 3282 # Gt Pt 
    voxel_theta = np.deg2rad(radar.voxel_theta)
    voxel_phi = np.deg2rad(radar.voxel_phi)

    tx_dist_loss_exponent = 0
    rx_dist_loss_exponent = 0

    P_incident = np.power(rho,2) * np.sin(voxel_theta) * voxel_phi * voxel_theta * (1/np.power(rho,tx_dist_loss_exponent)) * K_sq # rho.^1.4

    

    #Material dependent qualtities
    if use_spec and use_diffused:
        R_sq = np.ones((len(angles),))*0.8 # metal 0.3
        R_sq[points[:,4]==0.03] = 0.2 # Concrete 
        R_sq[points[:,4]==0.001] = 0.2 # Vegetation 
        R_sq[points[:,4]==0] = 0.1 # Road
        S_sq = 1 - R_sq # Scattering coefficient

    elif not use_spec:
        R_sq = np.zeros((len(angles),))
        S_sq = 1 - R_sq # Scattering coefficient

    elif not use_diffused:
        R_sq = np.ones((len(angles),))
        S_sq = 1 - R_sq # Scattering coefficient

    else:
        print("Both Scatter and specular can't be false")
        AssertionError

    P_absorbed_fac = np.zeros((len(angles),)) # metal
    P_absorbed_fac[points[:,4]==0.03] = 0.8 # concrete
    P_absorbed_fac[points[:,4]==0.001] = 0.8 # vegetation
    P_absorbed_fac[points[:,4]==0] = 1 # road
    

    P_reflected = P_incident*(1 - P_absorbed_fac)

    spec_exp = np.ones((len(angles),))*600
    spec_exp[points[:,4]==0.1] = 200 #300 # metal
    
    normalization = np.ones((len(angles),))*(1/0.10229)
    normalization[points[:,4]==0.1] = (1/0.17702) # metal
    


    P_spec = P_reflected * R_sq
    P_scat = P_reflected * S_sq

    ##find power in the incident direction now
    # pdb.set_trace()
    # P_spec_inc[points[:,4]==0.1] = points[points[:,4]==0.1,4]*0.15*np.abs(np.cos(angles[points[:,4]==0.1]))**300
    # P_spec_inc[points[:,4]!=0.1] = points[points[:,4]!=0.1,4]*0.6*np.abs(np.cos(angles[points[:,4]!=0.1]))**600

    P_spec_inc = normalization*(np.abs(np.cos(angles))**spec_exp)*(1/np.power(rho,rx_dist_loss_exponent))
    # P_spec_inc[points[:,4]!=0.1] = (1/0.10229)*(np.abs(np.cos(angles[points[:,4]==0.1]))**600)*(1/np.power(rho,2))

    P_spec_inc = P_spec_inc * P_spec

    #Lambertian
    P_scat_inc = P_scat* ((0.5*(1+abs(np.cos(angles))))**0.1)*(1/3.07385) * (1/np.power(rho,rx_dist_loss_exponent)) # lambda sqaure aperture taken care in heatmap
    
    P_spec_inc = P_spec_inc * 10e2 #30e2 #0.8e2
    P_scat_inc = P_scat_inc * 10e2 #30e2

    loss = P_spec_inc + P_scat_inc

    return loss, P_scat_inc, P_spec_inc

def get_loss_3(points, rho, elev_angle, angles, radar, use_spec = True, use_diffused = True, no_material = False):
    '''
    Note: angles should be used for power calculation as it is in reflecting surface coordinate system while theta is in radar coordinate system

    Implemement power conservation here:
    P_inc = P_return(rough_loss^2 + S^2) + P_trans
    P_scatter = P_return*S^2
    P_return = P_inc * R^2  (R -> reflection co-eff from fresnel equations)

    S = sqrt(1 - rough_loss^2)

    
    R_TE = cos(incd_angle) - sqrt(perm - sin(incd_angle)^2) / cos(incd_angle) + sqrt(perm - sin(incd_angle)^2)
    R_TM = perm*cos(incd_angle) - sqrt(perm - sin(incd_angle)^2) / perm*cos(incd_angle) + sqrt(perm - sin(incd_angle)^2)
    R^2 = (R_TE^2 + R_TM^2)/2

    Scatter_loss = R^2 * S^2 = R^2 * (1- rough_loss^2)
    rough_loss = exp(-0.5((4*pi*std_rough*cos(inc_angle)/lambda)^2))

    backscatter_lobe_loss = E_so^2 * (lobe_frac*cos(incd_angle)^2*alpha_R + (1-lobe_frac))
    '''

    # standard deviation of surface roughness
    unlabel_roughness = 0
    wood_roughness = 0.0017 #1.7mm
    conc_roughness = 0.0017 #1.7mm
    human_roughness = 0.01 # 100um
    metal_roughness = 0.00005 # 100um 
    
    roughness = np.array([unlabel_roughness,wood_roughness,conc_roughness,human_roughness,metal_roughness])

    unlabel_perm = 1
    wood_perm = 2 
    conc_perm = 5.24 
    human_perm = 1#15 
    metal_perm = 100000
    
    permittivity = np.array([unlabel_perm,wood_perm,conc_perm,human_perm,metal_perm])

    scat_normalization = 1/1.09*np.ones((len(angles),)) #*(1/0.10229) #####?????????#### this should bring the Eso^2 #####?????????####
    lobe_frac = 0.9 # ratio of energy in main_lobe/back_scatter_lobe
    spec_lobe_exp = 4.0 #lobe exponential along specular direction

    K_sq = 3282 # Gt Pt #####?????????#### is this from the peak amplitude calculation #####?????????####
    voxel_theta = np.deg2rad(radar.voxel_theta)
    voxel_phi = np.deg2rad(radar.voxel_phi)

    tx_dist_loss_exponent = 2
    rx_dist_loss_exponent = 0

    spec_angle_thresh = 2*np.pi/180#*(1/rho)

    P_incident = np.power(rho,2) * np.sin(elev_angle) * voxel_phi * voxel_theta * (1/np.power(rho,tx_dist_loss_exponent)) * K_sq 

    material = np.array(points[:,4])
    material = np.asarray(material, dtype = 'int')
    # material[:] = 3

    #Material dependent qualtities
    # if use_spec and use_diffused:
    R_TE_sq = np.power(((abs(np.cos(angles)) - np.sqrt(permittivity[material] - np.power(np.sin(angles),2)))/(abs(np.cos(angles)) + np.sqrt(permittivity[material] - np.power(np.sin(angles),2)))),2)
    R_TM_sq = np.power(((permittivity[material]*abs(np.cos(angles)) - np.sqrt(permittivity[material] - np.power(np.sin(angles),2)))/(permittivity[material]*abs(np.cos(angles)) + np.sqrt(permittivity[material] - np.power(np.sin(angles),2)))),2)
    R_sq = R_TE_sq #0.5 * (R_TE_sq + R_TM_sq)
    rough_sq = np.exp(-0.5*np.power(4*np.pi*roughness[material]*abs(np.cos(angles))*radar.f/radar.c,2))
    # if len(rough_sq[abs(angles) < spec_angle_thresh])!=0:
    #     print(f"Materials = {np.unique(material)}")
    #     print(f"R value max= {np.max(rough_sq[abs(angles) < spec_angle_thresh])}; average = {np.mean(rough_sq[abs(angles) < spec_angle_thresh])}")

    S_sq = 1 - rough_sq # Scattering coefficient = P_scat/P_reflec
    P_reflected = P_incident * R_sq
    P_scat = P_reflected * S_sq
    P_scat_lobe = scat_normalization*(lobe_frac * np.power(abs(np.cos(angles)),2*spec_lobe_exp) + (1-lobe_frac))
    P_scat = P_scat * np.power(P_scat_lobe,2) #lobe shape is for amplitude, need to square for power

    P_spec = P_reflected * rough_sq 
    P_spec = P_spec * np.ones((len(angles),)) * (abs(angles) < spec_angle_thresh) # specular reflection only if angle < 2 degrees

    if not use_spec:
        # R_TE_sq = np.power(((abs(np.cos(angles)) - np.sqrt(permittivity[material] - np.power(np.sin(angles),2)))/(abs(np.cos(angles)) + np.sqrt(permittivity[material] - np.power(np.sin(angles),2)))),2)
        # R_TM_sq = np.power(((permittivity[material]*abs(np.cos(angles)) - np.sqrt(permittivity[material] - np.power(np.sin(angles),2)))/(permittivity[material]*abs(np.cos(angles)) + np.sqrt(permittivity[material] - np.power(np.sin(angles),2)))),2)
        # R_sq = R_TE_sq #0.5 * (R_TE_sq + R_TM_sq)
        # rough_sq = np.exp(-0.5*np.power(4*np.pi*roughness[material]*abs(np.cos(angles))*radar.f/radar.c,2))

        # S_sq = 1 - rough_sq # Scattering coefficient = P_scat/P_reflec
        # P_reflected = P_incident * R_sq
        # P_scat = P_reflected * S_sq
        # P_scat_lobe = scat_normalization*(lobe_frac * np.power(abs(np.cos(angles)),2*spec_lobe_exp) + (1-lobe_frac))
        # P_scat = P_scat * np.power(P_scat_lobe,2) #lobe shape is for amplitude, need to square for power

        P_spec = np.zeros((len(angles),))

    elif not use_diffused:
        # R_TE_sq = np.power(((abs(np.cos(angles)) - np.sqrt(permittivity[material] - np.power(np.sin(angles),2)))/(abs(np.cos(angles)) + np.sqrt(permittivity[material] - np.power(np.sin(angles),2)))),2)
        # R_TM_sq = np.power(((permittivity[material]*abs(np.cos(angles)) - np.sqrt(permittivity[material] - np.power(np.sin(angles),2)))/(permittivity[material]*abs(np.cos(angles)) + np.sqrt(permittivity[material] - np.power(np.sin(angles),2)))),2)
        # R_sq = R_TE_sq #0.5 * (R_TE_sq + R_TM_sq)
        # rough_sq = np.exp(-0.5*np.power(4*np.pi*roughness[material]*abs(np.cos(angles))*radar.f/radar.c,2))

        # S_sq = 1 - rough_sq # Scattering coefficient = P_scat/P_reflec
        # P_reflected = P_incident * R_sq
        # P_scat = P_reflected * S_sq
        # P_scat_lobe = scat_normalization*(lobe_frac * np.power(abs(np.cos(angles)),2*spec_lobe_exp) + (1-lobe_frac))
        # P_scat = P_scat * np.power(P_scat_lobe,2) #lobe shape is for amplitude, need to square for power

        P_scat = np.zeros((len(angles),))

        # P_spec = P_reflected * rough_sq 
        # P_spec = P_spec * np.ones((len(angles),)) * (abs(angles) < 2*np.pi/180) #####?????????#### specular reflection only if angle < 2 degrees #####?????????####

    elif not use_spec and not use_diffused:
        print("Both Scatter and specular can't be false")
        AssertionError

    P_spec = P_spec *4*100*25/9
    P_scat = P_scat *100*25/9

    loss = P_spec + P_scat
    # loss = loss * 2**7.2

    # fig, ax = plt.subplots(1,7,  figsize=(10,5))
    # ax[0].plot(loss)
    # ax[0].set_title("total loss",fontsize=10)
    # ax[1].plot(P_spec)
    # ax[1].set_title("specular loss",fontsize=10)
    # ax[2].plot(P_scat)
    # ax[2].set_title("scatter loss",fontsize=10)
    # ax[3].plot(material)
    # ax[3].set_title("material",fontsize=10)
    # ax[4].plot(angles)
    # ax[4].set_title("angles",fontsize=10)
    # ax[5].plot(R_TE_sq)
    # ax[5].set_title("R TE",fontsize=10)
    # ax[6].plot(R_TM_sq) #abs(numerator)/abs(denominator))
    # ax[6].set_title("R TM",fontsize=10)

    # plt.show()
    # plt.suptitle('loss profile',fontsize=15, y=1)
    # plt.savefig('loss_comp_sim.jpg')

    return loss, P_scat, P_spec


if __name__ == '__main__':

    # dummy_points = np.array([[0,0,0,0,0.1], #
    #                         [0,0,0,0,0.03],
    #                         [0,0,0,0,0.006]])
    

    dummy_points = np.vstack((np.tile([0,0,0,0,0.1],(180,1)),np.tile([0,0,0,0,0.03],(180,1)),np.tile([0,0,0,0,0.001],(180,1))))
    latest_dummy_points = np.vstack((np.tile([0,0,0,0,4],(180,1)),np.tile([0,0,0,0,2],(180,1)),np.tile([0,0,0,0,1],(180,1))))
    
    dummy_rho = np.array([20]*540)

    dummy_angles = np.hstack((np.linspace(-np.pi/2,np.pi/2,180),np.linspace(-np.pi/2,np.pi/2,180),np.linspace(-np.pi/2,np.pi/2,180)))

    radar = radar()

    #loss_old, scat_old, spec_old = get_loss(dummy_points,dummy_rho,dummy_angles,radar)
    loss_new, scat_new, spec_new = get_loss_2(dummy_points,dummy_rho,dummy_angles,radar)
    loss_latest, scat_latest, spec_latest = get_loss_3(latest_dummy_points,dummy_rho,dummy_angles,radar)
    

    # print(f"Old Loss: {loss_old}")
    # print(f"New Loss: {loss_new}")

    fig, ax = plt.subplots(2,3,  figsize=(10,5))

    #ax[0,0].plot(loss_old)
    #ax[0,1].plot(spec_old)
    #ax[0,2].plot(scat_old)
    ax[0,0].plot(loss_latest)
    ax[0,0].set_title("New total loss",fontsize=10)
    ax[0,1].plot(spec_latest)
    ax[0,1].set_title("New specular loss",fontsize=10)
    ax[0,2].plot(scat_latest)
    ax[0,2].set_title("New scatter loss",fontsize=10)
    ax[1,0].plot(loss_new)
    ax[1,0].set_title("Prev total loss",fontsize=10)
    ax[1,1].plot(spec_new)
    ax[1,1].set_title("Prev specular loss",fontsize=10)
    ax[1,2].plot(scat_new)
    ax[1,2].set_title("Prev scatter loss",fontsize=10)

    plt.show()
    plt.suptitle('0:179 => Metal 180:359 =>Concrete 360:539=>Wood',fontsize=15, y=1)
    plt.savefig('loss_comparison.jpg')

