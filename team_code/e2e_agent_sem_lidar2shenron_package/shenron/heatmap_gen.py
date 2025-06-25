import numpy as np
from e2e_agent_sem_lidar2shenron_package.ConfigureRadar import radar
import matplotlib.pyplot as plt
from matplotlib import cm
from mat4py import loadmat
import math
from mpl_toolkits.mplot3d import axes3d, Axes3D
import time
import scipy.io as sio
from numba import jit
from numba import int32, float64, complex128
from noise_utils import get_noise
import pdb

# @jit(nopython=True)
def heatmap_gen(rho, theta, loss, speed, radar, plot_fig, return_power):

    range_res = radar.c / (2 * radar.B)
    max_range = range_res * radar.N_sample

    Ts = 1 / radar.samp_rate
    t = np.arange(0, radar.chirpT, Ts)
    tau_resolution = 1 / radar.B
    k = radar.B / radar.chirpT
    x = np.exp(1j * 2 * np.pi * (radar.f + 0.5 * k * t) * t)
    
    # noise_prop = loadmat('noise.mat')
    # real_fft_ns = np.random.normal(noise_prop['noise_mean_real'], noise_prop['noise_std_real']).T
    # complex_fft_ns = np.random.normal(noise_prop['noise_mean_real'], noise_prop['noise_std_real']).T
    # final_noise = real_fft_ns + 1j * complex_fft_ns
    # signal_Noisy = np.fft.ifft(final_noise, radar.N_sample, 1) * 10
    signal_Noisy = get_noise(radar)

    if radar.doppler_mode:
        measurement = np.zeros((radar.chirps, radar.nRx, radar.N_sample), dtype="complex128")  # doppler,AoA,range
        # start = time.time()
        for chirp in range(radar.chirps):
            tau = 2 * rho / radar.c + chirp * radar.chirpT * speed / radar.c
            _lambda = radar.c / radar.f
            sRx = _lambda / 2
            _lambda = radar.c / radar.f
            tau_vec = np.zeros((radar.nRx, len(rho)))
            for i in range(radar.nRx):
                tau_vec[i, :] = tau + i * sRx * np.sin(np.pi / 2 - theta) / radar.c

            sum_samp = np.zeros((radar.nRx, radar.N_sample), dtype="complex128")
            for j in range(rho.shape[0]):
                if return_power:
                    if rho[j] != 0:
                        signal = np.sqrt(loss[j]) * np.exp(
                            1j * 2 * np.pi * (radar.f + 0.5 * k * (np.expand_dims(t,0) - np.expand_dims(tau_vec[:,j],1))) * (
                                    np.expand_dims(t,0) - np.expand_dims(tau_vec[:,j],1)))
                else:
                    if rho[j] != 0:
                        signal = loss[j] * (1 / rho[j] ** 2) * np.exp(
                            1j * 2 * np.pi * (radar.f + 0.5 * k * (np.expand_dims(t,0) - np.expand_dims(tau_vec[:,j],1))) * (
                                    np.expand_dims(t,0) - np.expand_dims(tau_vec[:,j],1)))

#                 noise_real = (1j*1j*-1) * (2 * np.random.rand(radar.nRx, radar.N_sample) - 1)
#                 noise_complex = (1j) * (2 * np.random.rand(radar.nRx, radar.N_sample) - 1)
#                 noise = (noise_real+noise_complex) * radar.noise_amp
#                 signal_Noisy = signal + noise

                sum_samp = sum_samp + signal
            adc_sampled = np.sqrt(radar.gain * _lambda ** 2 / (4 * np.pi) ** 3) * np.conj(sum_samp) * (x)
            adc_sampled = adc_sampled + signal_Noisy
            measurement[chirp, :, :] = adc_sampled

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

#                 noise_real = (1j*1j*-1) * (2 * np.random.rand(radar.nRx, radar.N_sample) - 1)
#                 noise_complex = (1j) * (2 * np.random.rand(radar.nRx, radar.N_sample) - 1)
#                 noise = (noise_real+noise_complex) * radar.noise_amp
#                 signal_Noisy = signal + noise

                sum_samp = sum_samp + signal

        adc_sampled = np.sqrt(radar.gain * _lambda ** 2 / (4 * np.pi) ** 3) * np.conj(sum_samp) * (x)
        adc_sampled = adc_sampled + signal_Noisy    
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

    test_WI=1
    if (test_WI):
        points = np.load("E:/Radar_sim/simlator/git repo/Heatmap-sim/simulation5.npy")
        rho = np.linalg.norm(points[:, 0:3] - np.array(radar.center+radar.elv), axis=1)
        theta = math.pi / 2 - np.arctan(((points[:, 0] - radar.center[0]) / (points[:, 1] - radar.center[1])))
        loss = points[:, 3]
        speed = np.zeros_like(rho)
    else:
        rho = np.array([5, 5, 10])  # np.linspace(7.9749, 9.2793, 97)
        theta = np.array([math.pi / 3, math.pi * 2 / 3, math.pi / 2])  # np.linspace(1.6856, 1.521, 97)
        loss = np.ones_like(rho)
        speed = np.zeros_like(rho)

    x_axis, y_axis, plot_data, doppler_data = heatmap_gen(rho, theta, loss, speed, radar, 1)

    diction = {"x_axis": x_axis, "y_axis": y_axis, "plot_data": plot_data}
    sio.savemat('test_heatmap.mat', diction)

    # fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
    # ax.view_init(elev=90, azim=180)
    # surf = ax.plot_surface(x_axis, y_axis, plot_data ** 0.7, cmap=cm.coolwarm,
    #                        linewidth=0, antialiased=False)
    # plt.show()

