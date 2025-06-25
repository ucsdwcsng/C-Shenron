import numpy as np
import yaml
import sys
import glob

# sys.path.append(glob.glob('../carla_garage_radar/team_code/sim_radar_utils/'))
sys.path.append('radar-imaging-dataset/carla_garage_radar/team_code/sim_radar_utils/')
# sys.path.append('radar-imaging-dataset/carla_garage_radar/team_code/')


# sys.path.append("/radar-imaging-dataset/mmfn_project/mmfn_scripts/team_code/sim_radar_utils/")

from scipy import signal
from scipy.fft import fft, fftshift
from sim_radar_utils.cfar_detector import CA_CFAR
from sim_radar_utils.utils_radar import *


cfarCfg = config['CFAR']


class RadarProcessor:
    def __init__(self):
        # radar data will be shaped as (# of chirp, # of sample, # of antenna)
        # self.rangeWin = np.tile(signal.windows.hann(radarCfg['N']), (radarCfg['Np'], len(radarCfg['AntIdx']), 1))
        self.rangeWin = np.tile(signal.windows.hann(radarCfg['N']), (radarCfg['Np'], 86, 1))
        self.rangeWin = np.transpose(self.rangeWin, (2, 0, 1))

        # self.velWin = np.tile(signal.windows.hann(radarCfg['Np']), (radarCfg['N'], len(radarCfg['AntIdx']), 1))
        self.velWin = np.tile(signal.windows.hann(radarCfg['Np']), (radarCfg['N'], 86, 1))
        self.velWin = np.transpose(self.velWin, (0, 2, 1))

        rangeRes = fftCfg['c0'] / (2*(radarCfg['fStop'] - radarCfg['fStrt']))
        self.rangeAxis = np.arange(0, fftCfg['NFFT'])*radarCfg['N']/fftCfg['NFFT']*rangeRes
        self.RMinIdx = np.argmin(np.abs(self.rangeAxis - fftCfg['RMin']))
        self.RMaxIdx = np.argmin(np.abs(self.rangeAxis - fftCfg['RMax']))
        self.rangeAxis = self.rangeAxis[self.RMinIdx:self.RMaxIdx]

        self.angleAxis = np.arcsin(2 * np.arange(-fftCfg['NFFTAnt']/2, fftCfg['NFFTAnt']/2) / fftCfg['NFFTAnt'])

        fc = (radarCfg['fStop'] + radarCfg['fStrt'])/2
        self.velAxis = np.arange(-fftCfg['NFFTVel']//2, fftCfg['NFFTVel']//2)/fftCfg['NFFTVel']*(1/radarCfg['Tp'])*fftCfg['c0']/(2*fc)

        self.cfar = CA_CFAR(win_param=cfarCfg['win_param'], 
                            threshold=cfarCfg['threshold'], 
                            rd_size=(self.RMaxIdx-self.RMinIdx, fftCfg['NFFTVel']))

    def cal_range_fft(self, data):
        '''apply range window and doppler window and apply fft on each sample to get range profile'''
        return fft(data * self.rangeWin * self.velWin, fftCfg['NFFT'], 0)

    def cal_doppler_fft(self, rangeProfile):
        '''apply fft on each chirp to get doppler profile'''
        return fftshift(fft(rangeProfile[self.RMinIdx:self.RMaxIdx+1, :], fftCfg['NFFTVel'], 1), 1)

    def cal_angle_fft(self, rangeProfile):
        '''# apply fft on each antenna to get angle profile'''
        return fftshift(fft(rangeProfile[self.RMinIdx:self.RMaxIdx+1, :], fftCfg['NFFTAnt'], 2), 2)

    def convert_to_pcd(self, dopplerProfile):
        avgDopplerProfile = np.squeeze(np.mean(dopplerProfile, 2))

        # detect useful peaks using CFAR
        detections = self.cfar(np.square(np.abs(avgDopplerProfile)))

        # identify range bin and velocity bin for each detected point
        rowSel, colSel = np.nonzero(detections)

        # pointSel = np.zeros(shape=(len(rowSel), len(radarCfg['AntIdx'])), dtype=complex)
        pointSel = np.zeros(shape=(len(rowSel), 86), dtype=complex)
        for i, (row, col) in enumerate(zip(rowSel, colSel)):
            pointSel[i] = dopplerProfile[row, col, :]

        # calcualte range and anlge value
        rangeVals = self.rangeAxis[rowSel]
        aoaProfile = fftshift(fft(pointSel, fftCfg['NFFTAnt'], 1), 1)
        angleIdx = np.argmax(np.abs(aoaProfile), axis=1)
        angleVals = self.angleAxis[angleIdx]

        rangeAoA = np.transpose(np.stack([rangeVals, angleVals]))

        # convert Range-AoA to pointcloud
        pointcloud = [rangeVals*np.cos(angleVals), rangeVals*np.sin(angleVals)]
        pointcloud = np.transpose(np.stack(pointcloud))

        return rangeAoA, pointcloud
