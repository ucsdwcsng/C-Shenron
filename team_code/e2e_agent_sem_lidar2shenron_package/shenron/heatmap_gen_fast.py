import numpy as np
from e2e_agent_sem_lidar2shenron_package.ConfigureRadar import radar
import matplotlib.pyplot as plt
from matplotlib import cm
from mat4py import loadmat
import math
from mpl_toolkits.mplot3d import axes3d, Axes3D
import time
import scipy.io as sio
# from numba import jit
# from numba import int32, float64, complex128
import pdb
import torch
from pynvml import *

def get_gpu_id_most_avlbl_mem():

    nvmlInit()
    deviceCount = nvmlDeviceGetCount()
    free = []
    for i in range(deviceCount):
        h = nvmlDeviceGetHandleByIndex(i)
        info = nvmlDeviceGetMemoryInfo(h)
        free.append(info.free)
    free = np.array(free)
    # h0 = nvmlDeviceGetHandleByIndex(0)
    # h1 = nvmlDeviceGetHandleByIndex(1)
    # h2 = nvmlDeviceGetHandleByIndex(2)
    # h3 = nvmlDeviceGetHandleByIndex(3)
    # info0 = nvmlDeviceGetMemoryInfo(h0)
    # info1 = nvmlDeviceGetMemoryInfo(h1)
    # info2 = nvmlDeviceGetMemoryInfo(h2)
    # info3 = nvmlDeviceGetMemoryInfo(h3)
    
    # free = np.array([info0.free,info1.free,info2.free,info3.free])
    
    return np.argmax(free), np.max(free)

# @jit(nopython=True)
def heatmap_gen(rho, theta, loss, speed, radar, plot_fig, return_power):
    start = time.time()
    range_res = radar.c / (2 * radar.B)
    max_range = range_res * radar.N_sample

    Ts = 1 / radar.samp_rate
    t = np.arange(0, radar.chirpT, Ts)
    tau_resolution = 1 / radar.B
    k = radar.B / radar.chirpT
    x = np.exp(1j * 2 * np.pi * (radar.f + 0.5 * k * t) * t)


    _lambda = radar.c / radar.f
    sRx = _lambda / 2
    _lambda = radar.c / radar.f

    # Declare if we should use cpu or cuda
    gpu_id, _ = get_gpu_id_most_avlbl_mem()
    # pdb.set_trace()
    device = torch.device(f'cuda:{gpu_id}' if torch.cuda.is_available() else 'cpu')
    # device = torch.device('cpu')
    print(f"------- Using {device}:{gpu_id} -------") 

    # beamforming_vector_constant = np.zeros((radar.nRx,rho.shape[0]))
    delta = torch.zeros((radar.nRx,rho.shape[0]),device= device)
    

    ##Noise
    # noise_prop = loadmat('noise.mat')
    # real_fft_ns = np.random.normal(noise_prop['noise_mean_real'], noise_prop['noise_std_real']).T
    # complex_fft_ns = np.random.normal(noise_prop['noise_mean_real'], noise_prop['noise_std_real']).T
    # final_noise = real_fft_ns + 1j * complex_fft_ns
    # signal_Noisy = np.fft.ifft(final_noise, radar.N_sample, 1) * 10
    signal_Noisy = radar.get_noise()

    ##initialising on torch
    loss = torch.tensor(loss*(1 / rho ** 2), device=device).float()
    rho = torch.tensor(rho, device=device).float()
    signal_Noisy = torch.tensor(signal_Noisy, device=device)
    theta = torch.tensor(theta, device=device).float()
    speed = torch.tensor(speed, device=device).float()
    t = torch.tensor(t, device=device).float()
    
    torch.set_printoptions(precision=10)
    if radar.doppler_mode:
        measurement = np.zeros((radar.chirps, radar.nRx, radar.N_sample), dtype="complex128")  # doppler,AoA,range
        # start = time.time()
        for chirp in range(radar.chirps):
            tau = 2 * (rho / radar.c + chirp * (radar.chirp_rep) * speed / radar.c)
            # tau = 2 * rho / radar.c + chirp * (12*27e-6) * speed / radar.c
            # print(tau[torch.argmax(abs(speed))])            
            for i in range(radar.nRx):
                delta[i,:] = i * sRx * torch.sin(np.pi / 2 - theta) / radar.c

            # delta = torch.tensor(delta,device='cuda')
            # tau = torch.tensor(tau,device='cuda')
            # t = torch.tensor(t,device='cuda')

            # beamforming_vector = np.exp(1j*2*np.pi*(radar.f*delta[:,:,None] - 0.5*k*delta[:,:,None]**2 - k*tau[None,:,None]*delta[:,:,None] + k*delta[:,:,None]@t[None,None,:])) #(80xN)
            beamforming_vector = torch.exp(1j*2*np.pi*(radar.f*delta[:,:,None] - 0.5*k*delta[:,:,None]**2 - k*tau[None,:,None]*delta[:,:,None] + k*delta[:,:,None]@t[None,None,:])) #(80xN)

            
            # tau = tau.cpu().numpy()
            # t = t.cpu().numpy()
            # beamforming_vector = beamforming_vector.cpu().numpy()
            

            dechirped = torch.exp((1j * 2 * np.pi) *(radar.f * tau[:,None] + k * tau[:,None]@t[None,:] - 0.5 * k * tau[:,None]**2))
            # loss_factor = (loss * (1 / rho ** 2))[:,None] ## from network
            loss_factor = (torch.sqrt(loss)[:,None]) ## from network

            signal_single_antenna = loss_factor*dechirped # (Nx256)

            signal = beamforming_vector*signal_single_antenna[None,:,:] # (80x256)

            signal = torch.squeeze(torch.sum(signal,1))

            # pdb.set_trace()

                    # (np.expand_dims(t,0) - np.expand_dims(tau_vec[:,j],1))) * (
                    #     np.expand_dims(t,0) - np.expand_dims(tau_vec[:,j],1)))

            # signal = np.exp(1j*2*np.pi * ())

            # noise_real = (1j*1j*-1) * (2 * np.random.rand(radar.nRx, radar.N_sample) - 1)
            # noise_complex = (1j) * (2 * np.random.rand(radar.nRx, radar.N_sample) - 1)
            # noise = (noise_real+noise_complex) * radar.noise_amp
            # signal_Noisy = signal

            # sum_samp = sum_samp + signal_Noisy ## Here I am not adding individual noise for each point
            


            # adc_sampled = np.sqrt(radar.gain * _lambda ** 2 / (4 * np.pi) ** 3) * np.conj(signal_Noisy) * (x)
            adc_sampled = torch.sqrt(torch.tensor(radar.gain * _lambda ** 2 / (4 * np.pi) ** 3)) * signal
            adc_sampled = adc_sampled + signal_Noisy
            
            adc_sampled = adc_sampled.cpu().numpy()

            

            measurement[chirp, :, :] = adc_sampled
        # pdb.set_trace()
        del rho
        del loss
        # del signal_noisy
        del speed
        del theta
        del t
        del dechirped
        del beamforming_vector
        del signal
        del signal_single_antenna
        with torch.cuda.device(f'cuda:{gpu_id}'):
            torch.cuda.empty_cache()
        # pdb.set_trace()
        return measurement
        # end = time.time()
        # diction = {"doppler_cube": measurement}
        # sio.savemat("E:/Radar_sim/simlator/git repo/Heatmap-sim/doppler_cube.mat", diction)
#         RangeFFT = np.fft.fft(measurement, radar.N_sample, 2)
#         AngleFFT = np.fft.fftshift(np.fft.fft(RangeFFT[0, :, :], radar.angle_fft_size, 0), 0)
#         Doppler_data = np.fft.fftshift(np.fft.fft(np.fft.fftshift(np.fft.fft(RangeFFT, radar.angle_fft_size, 1), 1),
#                                                   radar.chirps, 0), 0)
#         Doppler_heatmap = np.sum(np.arange(radar.chirps)[:, None, None] * np.abs(Doppler_data), axis=0) / np.sum(
#             np.abs(Doppler_data), axis=0) - radar.chirps / 2
            
    else:
        tau = 2 * rho / radar.c
        _lambda = radar.c / radar.f
        sRx = _lambda / 2  # separation
        _lambda = radar.c / radar.f

        tau_vec = np.zeros((radar.nRx, rho.shape[0]))
        for i in range(radar.nRx):
            tau_vec[i, :] = tau + (i) * sRx * np.sin(np.pi / 2 - theta) / radar.c

        sum_samp = np.zeros((radar.nRx, radar.N_sample), dtype="complex128")
        for j in range(rho.shape[0]):
            if (rho[j] != 0):
                if return_power:
                    if rho[j] != 0:
                        signal = 10**10 * np.sqrt(loss[j]) * np.exp(
                            1j * 2 * np.pi * (radar.f + 0.5 * k * (np.expand_dims(t,0) - np.expand_dims(tau_vec[:,j],1))) * (
                                    np.expand_dims(t,0) - np.expand_dims(tau_vec[:,j],1)))
                else:
                    if rho[j] != 0:
                        signal = loss[j] * (1 / rho[j] ** 2) * np.exp(
                            1j * 2 * np.pi * (radar.f + 0.5 * k * (np.expand_dims(t,0) - np.expand_dims(tau_vec[:,j],1))) * (
                                    np.expand_dims(t,0) - np.expand_dims(tau_vec[:,j],1)))

                noise_real = (1j*1j*-1) * (2 * np.random.rand(radar.nRx, radar.N_sample) - 1)
                noise_complex = (1j) * (2 * np.random.rand(radar.nRx, radar.N_sample) - 1)
                noise = (noise_real+noise_complex) * radar.noise_amp
                signal_Noisy = signal + noise

                sum_samp = sum_samp + signal_Noisy*10**5

        adc_sampled = np.sqrt(radar.gain * _lambda ** 2 / (4 * np.pi) ** 3) * np.conj(sum_samp) * (x)
        # RangeFFT = np.fft.fft(adc_sampled, radar.N_sample, 1)

        # pwr_prof = 10*np.log10(np.sum(abs(RangeFFT)**2, 0)+1)
        # plt.plot(radar.range_res*np.arange(radar.N_sample), pwr_prof)
        # plt.axis([0, 256*radar.range_res, 70, 180])
        # plt.show()

        # AngleFFT = np.fft.fftshift(np.fft.fft(RangeFFT, radar.angle_fft_size, 0), 0)

        # Doppler_heatmap = np.zeros(np.shape(AngleFFT))

        return measurement


    # print('runtime is ', end - start)

#     d = 1
#     sine_theta = -2 * np.arange(-radar.angle_fft_size / 2, (radar.angle_fft_size / 2) + 1) / radar.angle_fft_size / d
#     # sine_theta = -2*np.arange(-radar.angle_fft_size/2,(radar.angle_fft_size/2)+1)/radar.angle_fft_size/d
#     cos_theta = np.sqrt(1 - sine_theta ** 2)
#     indices_1D = np.arange(0, radar.N_sample)
#     [R_mat, sine_theta_mat] = np.meshgrid(indices_1D * range_res, sine_theta)
#     [_, cos_theta_mat] = np.meshgrid(indices_1D, cos_theta)

#     x_axis = R_mat * cos_theta_mat
#     y_axis = R_mat * sine_theta_mat
#     mag_data_static = abs(np.vstack(
#         (AngleFFT, AngleFFT[255, :])))  # np.column_stack((abs(AngleFFT[indices_1D,:]),abs(AngleFFT[indices_1D,0])))
#     mag_data_doppler = abs(np.vstack((Doppler_heatmap, Doppler_heatmap[255, :])))
#     # doppler_cube = np.concatenate((Doppler_data, Doppler_data[:, 255, :][:, np.newaxis, :]), 1)

#     mag_data_static = np.flipud(mag_data_static)
#     mag_data_doppler = np.flipud(mag_data_doppler)
#     # doppler_cube = np.flipud(doppler_cube)

#     return x_axis, y_axis, mag_data_static, mag_data_doppler
       

if __name__ == '__main__':
    radar = radar()
    radar.chirps = 1
    radar.center = np.array([0.0, 0.0])  # center of radar
    radar.elv = np.array([0.0])

    test_WI=1
    if (test_WI):
        # points = np.load("E:/Radar_sim/simlator/git repo/Heatmap-sim/simulation5.npy")
        cell_array = sio.loadmat("wi_data/single_radar_wi_10m_allangles.mat")
        
        for i in range(36):
            print(f"Simulation: {i}")
            points = cell_array['cell_array'][i][0]
            if points.shape[0]==0:
                print("Skipped")
                continue
            rho = np.linalg.norm(points[:, 0:3], axis=1)
            theta = math.pi / 2 - np.arctan(((points[:, 0] - radar.center[0]) / (points[:, 1] - radar.center[1])))
            loss = 10**(points[:, 3]/20)
            speed = np.zeros_like(rho)
    

            adc_data = heatmap_gen(rho, theta, loss, speed, radar, 1, 0)
            diction = {"adc_data": adc_data}
            sio.savemat(f"wi_data/simulation_{i}.mat", diction)

    # fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
    # ax.view_init(elev=90, azim=180)
    # surf = ax.plot_surface(x_axis, y_axis, plot_data ** 0.7, cmap=cm.coolwarm,
    #                        linewidth=0, antialiased=False)
    # plt.show()


