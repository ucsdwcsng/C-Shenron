import numpy as np
import matplotlib.pyplot as plt
import torch
import pdb
from scipy import signal
from scipy import io as sio
import matplotlib.pyplot as plt

def separate_real_imag(input):

    real = input.real
    imag = input.imag

    out_tensor = np.stack((real,imag), axis=0)

    return out_tensor

def separate_mag_phase(input):

    mag = np.abs(input)
    phase = np.angle(input)

    out_tensor = np.stack((mag,phase), axis=0)

    return out_tensor

def convert_adc_to_3d_fft(adcData, rangefft_size = 256, dopplerfft_size=128, anglefft_size = 256, window=False, distance_null = False):
    """
    Function to convert adc data to RA tensor
    Input: 
        adcData: 
            NxDxM -- where N is the range dimension and M is the angle dimension and D is doppler.

    Output:
        RA tensor: R-size x A-size
    """
    if window:
        range_hanning = np.hanning(rangefft_size)[:,None,None]
        range_hanning = np.tile(range_hanning, (1,adcData.shape[1],adcData.shape[2]))

        adcData = adcData*range_hanning
    
    if distance_null:
        radar_range_res = 3e8/(2*2.56e8)
        range_vector = np.arange(rangefft_size)*radar_range_res
        range_vector_loss = range_vector**4
        range_vector_tiled = np.tile(range_vector_loss[:,None, None], (1,adcData.shape[1],adcData.shape[2]))
        
        adcData = adcData*range_vector_tiled
    rangefft = np.fft.fft(adcData, rangefft_size, 0)
    dopplerfft = np.fft.fft(rangefft, dopplerfft_size, 1)
    anglefft = np.fft.fftshift(np.fft.fft(dopplerfft, anglefft_size, 2),2)

    # self.anglefft = anglefft
    return anglefft

def cart2polar(x, y, in_pixels, limit, range_res):
    
    #Assuming uniform theta resolution
    r = np.sqrt((limit-x)**2 + (y)**2)*(1/range_res)
    
    theta = np.arctan2((y),(limit-x))
    theta_px = np.sin(theta)*(in_pixels/2) + in_pixels/2 
    return r, theta_px

def polar_to_cart(RATensor, range_res = 150/256, in_pixels = 256, limit = 150, out_pixels = 256):

    """
    convert a polar range angle tensor to cartesian array
    Input: 
        RATensor: NxM
    Output:
        cart_RATensor: NxM
    """  

    X, Y = np.meshgrid(np.linspace(0, limit, out_pixels), 
                            np.linspace(-limit/2, limit/2, out_pixels))

    R_samp, Theta_samp = cart2polar(X, Y, in_pixels, limit, range_res)

    R_samp = R_samp.astype(int)
    Theta_samp = Theta_samp.astype(int)
    
    R_samp[(R_samp>(in_pixels-1))] = 0
    R_samp[(R_samp<0)] = 0
    Theta_samp[(Theta_samp>(in_pixels-1))] = 0
    Theta_samp[(Theta_samp<0)] = 0
    
    
    if RATensor.ndim >2:
        polar_img = RATensor[...,R_samp,Theta_samp]
        # polar_img = torch.reshape(polar_img,(RATensor.shape[0],RATensor.shape[1],out_pixels,out_pixels))
        if torch.is_tensor(RATensor):
            polar_img = torch.transpose(polar_img,-1,-2)
        else:
            polar_img = np.swapaxes(polar_img,-2,-1)
    else:
        polar_img = RATensor[R_samp,Theta_samp]
        polar_img = torch.reshape(polar_img,(out_pixels,out_pixels))
        polar_img = polar_img.T

    return polar_img



def plot_data(data, ax = None):

    # if not ax:
    #     fig, ax = plt.subplots(1, len(data))

    for num, im in enumerate(data):
        ax[num].imshow(im)
    
    # plt.show()

    return ax
    # plt.imshow(polar_image[0,0])
    # plt.figure()
    # plt.imshow(cart_image[0,0])

def CFAR_filtered_output():
    ##Cfar layers
    guard_cell = 5
    CFAR_cell = 11
    guard_avg = nn.AvgPool2d(guard_cell, stride = 1, padding = 2, divisor_override = 1, count_include_pad=False)
    CFAR_avg = nn.AvgPool2d(CFAR_cell, stride = 1, padding = 5, divisor_override=1, count_include_pad=False)

    ##Input
    data = sio.loadmat("lid2rad10.mat")
    a = data["plot_data"]
    x_axis = data["x_axis"]
    y_axis = data["y_axis"]

    (size1,size2) = x_axis.shape

    ##CFAR processing
    a = torch.tensor(a,dtype=torch.float)

    b = guard_avg(a[None,:,:])
    c = CFAR_avg(a[None,:,:])

    CA_region = (c-b)/(CFAR_cell**2-guard_cell**2)
    thrshold = 10

    cfar_indices = np.squeeze(a)>np.squeeze(CA_region)*thrshold
    cfar_indices[[0,1,2,size2-3,size2-2,size2-1],:] = 0
    cfar_indices[:,[0,1,2,size2-3,size2-2,size2-1]] = 0

    imag = np.zeros((size1,size2))
    imag[cfar_indices] = 1

    x_points = x_axis[cfar_indices]
    y_points = y_axis[cfar_indices]
    z_points = np.zeros_like(y_points)

    pc = np.vstack((x_points,y_points,z_points))
    pc = pc.T

    ##Plotting
    pcd=open3d.open3d.geometry.PointCloud()
    pcd.points= open3d.open3d.utility.Vector3dVector(pc)
    open3d.open3d.visualization.draw_geometries([pcd])

    fig, ax = plt.subplots(1,5)
    ax[0].imshow(a.numpy())
    ax[1].imshow(np.squeeze(b).numpy())
    ax[2].imshow(np.squeeze(c).numpy())
    ax[3].imshow(np.squeeze(CA_region).numpy())
    ax[4].imshow(imag)
    plt.show()
    
class CA_CFAR():
    """
    Description:
    ------------
        Cell Averaging - Constant False Alarm Rate algorithm
        Performs an automatic detection on the input range-Doppler matrix with an adaptive thresholding.
        The threshold level is determined for each cell in the range-Doppler map with the estimation
        of the power level of its surrounding noise. The average power of the noise is estimated on a
        rectangular window, that is defined around the CUT (Cell Under Test). In order the mitigate the effect
        of the target reflection energy spreading some cells are left out from the calculation in the immediate
        vicinity of the CUT. These cells are the guard cells.
        The size of the estimation window and guard window can be set with the win_param parameter.
    Implementation notes:
    ---------------------
        Implementation based on https://github.com/petotamas/APRiL
    Parameters:
    -----------
    :param win_param: Parameters of the noise power estimation window
                      [Est. window length, Est. window width, Guard window length, Guard window width]
    :param threshold: Threshold level above the estimated average noise power
    :type win_param: python list with 4 elements
    :type threshold: float
    Return values:
    --------------
    """

    def __init__(self, win_param, threshold, rd_size):
        win_width = win_param[0]
        win_height = win_param[1]
        guard_width = win_param[2]
        guard_height = win_param[3]

        # Create window mask with guard cells
        self.mask = np.ones((2 * win_height + 1, 2 * win_width + 1), dtype=bool)
        self.mask[win_height - guard_height:win_height + 1 + guard_height, win_width - guard_width:win_width + 1 + guard_width] = 0

        # Convert threshold value
        self.threshold = 10 ** (threshold / 10)

        # Number cells within window around CUT; used for averaging operation.
        self.num_valid_cells_in_window = signal.convolve2d(np.ones(rd_size, dtype=float), self.mask, mode='same')

    def __call__(self, rd_matrix):
        """
        Description:
        ------------
            Performs the automatic detection on the input range-Doppler matrix.
        Implementation notes:
        ---------------------
        Parameters:
        -----------
        :param rd_matrix: Range-Doppler map on which the automatic detection should be performed
        :type rd_matrix: R x D complex numpy array
        Return values:
        --------------
        :return hit_matrix: Calculated hit matrix
        """
        # Convert range-Doppler map values to power
        rd_matrix = np.abs(rd_matrix) ** 2

        # Perform detection
        rd_windowed_sum = signal.convolve2d(rd_matrix, self.mask, mode='same')
        rd_avg_noise_power = rd_windowed_sum / self.num_valid_cells_in_window
        rd_snr = rd_matrix / rd_avg_noise_power
        hit_matrix = rd_snr > self.threshold

        return hit_matrix


if __name__ == "__main__":

    sim_adc = sio.loadmat("/home/Kshitiz/semantic_lidar/radar/0099.mat")
    sim_adc_data = sim_adc['adc_data']
    sim_adc_data = np.transpose(sim_adc_data, (2,0,1))
    
    rangefft_size = 256
    anglefft_size = 256
    dopplerfft_size = 128
    
    sim_RDA = convert_adc_to_3d_fft(sim_adc_data ,rangefft_size,dopplerfft_size, anglefft_size, distance_null=False)

    sim_dra = np.transpose(sim_RDA,(1,0,2))
    
    limit = 50
    
    sim_dra_cart = abs(polar_to_cart(sim_dra,limit=limit,out_pixels=256))

    fig ,ax = plt.subplots(1,2)

    ax[0].imshow(abs(sim_dra[0,:,:]))
    ax[1].imshow(abs(sim_dra_cart[0,:,:]))

    plt.show()