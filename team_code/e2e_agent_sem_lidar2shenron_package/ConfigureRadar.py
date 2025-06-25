import numpy as np
# from numba.experimental import jitclass
from mat4py import loadmat
import pdb

class radar():

    """
    class to define the radar object with it's settings and to extract time intervals for updating the scene.
    Also contains voxel filter specs
    """
    def __init__(self, radartype):
        self.radartype = radartype
        if radartype == "ti_cascade":
            self.center = np.array([0.0, 4.0])  # center of radar
            self.elv = np.array([0.5])  # self position in z-axis
            self.orientation = 90  # orientation of radar

            self.f = 77e9
            self.B = 0.256e9  # Bandwidth
            self.c = 3e8
            self.N_sample = 256
            self.samp_rate = 15e6
            self.doppler_mode = 1
            self.chirps = 3  # 128
            self.nRx = 86 #16  # number of antennas(virtual antennas included, AOA dim)
            self.noise_amp = 0.005  #0.0001(concrete+metal)  # 0.00001(metal) #0.005(after skyward data)
            self.gain = 10 ** (105 / 10)  #190(concrete+metal)  # 210(metal)
            self.angle_fft_size = 256

            self.range_res = self.c / (2 * self.B)  # range resolution
            self.max_range = self.range_res * self.N_sample

            self.idle = 0 ## Idle time
            self.chirpT = self.N_sample / self.samp_rate  ## Time of chirp
            self.chirp_rep = 12*27e-6

            Ts = 1 / self.samp_rate
            self.t = np.arange(0, self.chirpT, Ts)
            self.tau_resolution = 1 / self.B
            self.k = self.B / self.chirpT

            self.voxel_theta = 2.0  # 0.5  # 0.1
            self.voxel_phi = 2.0  # 0.5  # 0.1
            self.voxel_rho = 0.05  # 0.1  # 0.05

        elif radartype == "radarbook":
            self.center = np.array([0.0, 4.0])  # center of radar
            self.elv = np.array([0.5])  # self position in z-axis
            self.orientation = 90  # orientation of radar

            self.f = 24e9
            self.B = 0.250e9  # Bandwidth
            self.c = 3e8
            self.N_sample = 256
            self.samp_rate = 1e6
            self.doppler_mode = 1
            self.chirps = 128
            self.nRx = 8  # number of antennas(virtual antennas included, AOA dim)
            self.noise_amp = 0.005  #0.0001(concrete+metal)  # 0.00001(metal) #0.005(after skyward data)
            self.gain = 10 ** (105 / 10)  #190(concrete+metal)  # 210(metal)
            self.angle_fft_size = 256

            self.range_res = self.c / (2 * self.B)  # range resolution
            self.max_range = self.range_res * self.N_sample

            
            self.chirpT = self.N_sample / self.samp_rate  ## Time of chirp
            self.chirp_rep = 0.75e-3
            self.idle = self.chirp_rep -  self.chirpT## Idle time

            Ts = 1 / self.samp_rate
            self.t = np.arange(0, self.chirpT, Ts)
            self.tau_resolution = 1 / self.B
            self.k = self.B / self.chirpT

            self.voxel_theta = 2  # 0.5  # 0.1
            self.voxel_phi = 2  # 0.5  # 0.1
            self.voxel_rho = 0.05  # 0.1  # 0.05

        else:
            raise Exception("Incorrect radartype selected")

    def get_noise(self):
        if self.radartype == "ti_cascade": 
            # noise_prop = loadmat('/radar-imaging-dataset/mmfn_project/mmfn_scripts/team_code/e2e_agent_sem_lidar2shenron_package/noise_data/noise_adc.mat')
            # real_fft_ns = np.random.normal(noise_prop['noise_mean_real'], noise_prop['noise_std_real']).T
            # complex_fft_ns = np.random.normal(noise_prop['noise_mean_real'], noise_prop['noise_std_real']).T
            # final_noise = real_fft_ns + 1j * complex_fft_ns
            # # signal_Noisy = np.fft.ifft(final_noise, radar.N_sample, 1) #* 10**4.5
            # # signal_Noisy = final_noise 
            # signal_Noisy = 0*final_noise 
            
            # for low resolution 16 channels
            signal_Noisy = np.random.normal(0,1,size=(self.nRx,self.N_sample))
            signal_Noisy = 0*(signal_Noisy + 1j*signal_Noisy)

        elif self.radartype == "radarbook":
            signal_Noisy = np.random.normal(0,1,size=(self.nRx,self.N_sample))
            signal_Noisy = 0*(signal_Noisy + 1j*signal_Noisy)

        else:
            raise Exception("Incorrect radartype selected")

        return signal_Noisy