import sys, os
import yaml
import numpy as np
# import open3d
from pyntcloud import PyntCloud
from PIL import Image
import matplotlib.pyplot as plt
import json


with open(os.path.join(os.path.abspath(os.path.dirname(__file__)), 'config.yaml'), 'r') as f:
	config = yaml.safe_load(f)
	radarCfg = config['radar']
	savePath = config['ROS']['SavePath']
	fftCfg = config['FFT']
	clstCfg = config['Cluster']
	radarCfg['fStrt'] = float(radarCfg['fStrt'])
	radarCfg['fStop'] = float(radarCfg['fStop']) 
	radarCfg['TRampUp'] = float(radarCfg['TRampUp'])
	radarCfg['TInt'] = float(radarCfg['TInt'])
	radarCfg['Tp'] = float(radarCfg['Tp'])
	radarCfg['IniTim'] = float(radarCfg['IniTim'])


def config_save_path(dirc):
	for node in savePath.keys():
		savePath[node] = dirc + savePath[node]


def create_folder(activeNode):
	for node in activeNode:
		if node != 'timeStamp' and node != 'radarCfg':
			os.makedirs(savePath[node], exist_ok=True)


def read_time_stamp():
	return np.load(savePath['timeStamp'])


def load_radar_cfg():
	with open(savePath['radarCfg'], 'r') as f:
		newCfg = json.load(f)
		for key in radarCfg.keys():
			radarCfg[key] = newCfg[key]

#reformat adc shenron
def reformat_adc_shenron(data):
	data = np.swapaxes(data, 1, 2)
	# print(data.shape)
	chirpLevelData = data
	# return np.transpose(chirpLevelData[:, :, radarCfg['AntIdx']], (1, 0, 2))
	return np.transpose(chirpLevelData[:, :, :], (1, 0, 2))


def reformat_adc(data):
	data = np.reshape(data, (radarCfg['Np'], len(radarCfg['TxSeq']), radarCfg['N'], radarCfg['NrChn']))
	chirpLevelData = data[:, 0, :, :]
	for j in range(1, len(radarCfg['TxSeq'])):
		chirpLevelData = np.concatenate((chirpLevelData, data[:, j, :, :]), axis=-1)
	return np.transpose(chirpLevelData[:, :, radarCfg['AntIdx']], (1, 0, 2))


def read_radar_data(frame):
	radarData = np.load(savePath['radarRaw'] + f'{frame:04}.npy')
	chirpLevelData = reformat_adc(radarData)
	return chirpLevelData

#read radar data shenron
def read_radar_data_shenron(frame):
	radarData = np.load(savePath['radarRaw'] + f'{frame:04}.npy')
	chirpLevelData = reformat_adc_shenron(radarData)
	return chirpLevelData


def read_radar_pcd(frame):
	points = np.load(savePath['radarPCD'] + f'{frame:04}.npy')
	return points


def read_camera_img(frame):
	rgbImg = np.asarray(Image.open(savePath['cameraRGB'] + f'{frame:04}.png'))
	depthImg = np.asarray(Image.open(savePath['cameraDepth'] + f'{frame:04}.png'))
	return rgbImg, depthImg


def read_lidar_pcd(frame):
	pcd = PyntCloud.from_file(savePath['lidarPCD'] + f'{frame:04}.pts')
	return pcd.xyz


def plot_range_doppler(vis, dopplerProfile, rangeAxis, velAxis):
	dopplerProfiledB = 20*np.log10(np.abs(dopplerProfile[:, :, 0]))
	dopplerProfileMax = np.max(dopplerProfiledB)
	dopplerProfileNorm = dopplerProfiledB - dopplerProfileMax
	dopplerProfileNorm[dopplerProfileNorm < -30] = -30
	vis.plot_depth_fig(
		data=dopplerProfileNorm,
		pos=[velAxis[0], fftCfg['RMin']], 
		scale=[(velAxis[-1] - velAxis[0])/fftCfg['NFFTVel'], (fftCfg['RMax'] - fftCfg['RMin'])/len(rangeAxis)]
	)
	

def plot_range_aoa(vis, aoaProfile, rangeAxis):
	aoaProfiledB = 20*np.log10(np.abs(aoaProfile[:, 1, :]))
	aoaProfileMax = np.max(aoaProfiledB)
	aoaProfileNorm = aoaProfiledB - aoaProfileMax
	aoaProfileNorm[aoaProfileNorm < -30] = -30
	vis.plot_depth_fig(
		data=aoaProfileNorm, 
		pos=[-1, fftCfg['RMin']],
		scale=[2.0/fftCfg['NFFTAnt']/np.pi*180, (fftCfg['RMax'] - fftCfg['RMin'])/len(rangeAxis)]
	)