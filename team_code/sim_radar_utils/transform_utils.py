import numpy as np
import torch

def get_range_angle(raw_radar_data):
    range_angle = np.abs(raw_radar_data).sum(axis=1)
    return range_angle

def cart2polar(x, y, in_pixels, range_res):
    # Dividing by range_res to get the range in pixels
    r = np.sqrt(x ** 2 + y ** 2) * (1 / range_res)
    
    #Assuming uniform theta resolution
    theta = np.arctan2(y, x)
    theta_px = np.sin(theta) * (in_pixels / 2) + in_pixels / 2 
    return r, theta_px

def polar_to_cart(RATensor, range_res = 150 / 256, in_pixels = 256, limit = 150, out_pixels = 256):

    """
    convert a polar range angle tensor to cartesian array
    Input: 
        RATensor: NxM
    Output:
        cart_RATensor: NxM
    """  

    X, Y = np.meshgrid(np.linspace(-limit/2, limit/2, out_pixels), np.linspace(-limit/2, limit/2, out_pixels))
    # Rotating the axes to make the plane point upwards
    X = np.rot90(X)
    Y = np.rot90(Y)
    
    R_samp, Theta_samp = cart2polar(X, Y, in_pixels, range_res)

    R_samp = R_samp.astype(int)
    Theta_samp = Theta_samp.astype(int)
    
    R_samp[(R_samp > (in_pixels - 1))] = in_pixels - 1
    R_samp[(R_samp < 0)] = 0
    Theta_samp[(Theta_samp > (in_pixels - 1))] = in_pixels - 1
    Theta_samp[(Theta_samp < 0)] = 0
    
    if RATensor.ndim >2:
        polar_img = RATensor[...,R_samp,Theta_samp]
    else:
        polar_img = RATensor[R_samp, Theta_samp]

    # Applying mask to extract the front-side view only
    if out_pixels % 2 == 0:
        slice_idx = int(out_pixels / 2)
    else:
        slice_idx = int(out_pixels / 2) + 1
    
    polar_img[slice_idx:, :] = 0
    
    return polar_img

# def polar_to_cart(RATensor, range_res = 150/256, in_pixels = 256, limit = 150, out_pixels = 256):

#     """
#     convert a polar range angle tensor to cartesian array
#     Input: 
#         RATensor: NxM
#     Output:
#         cart_RATensor: NxM
#     """  

#     X, Y = np.meshgrid(np.linspace(0, limit, out_pixels), np.linspace(-limit/2, limit/2, out_pixels))

#     R_samp, Theta_samp = cart2polar(X, Y, in_pixels, range_res)

#     R_samp = R_samp.astype(int)
#     Theta_samp = Theta_samp.astype(int)
    
#     R_samp[(R_samp>(in_pixels-1))] = 0
#     R_samp[(R_samp<0)] = 0
#     Theta_samp[(Theta_samp>(in_pixels-1))] = 0
#     Theta_samp[(Theta_samp<0)] = 0
    
#     if RATensor.ndim >2:
#         polar_img = RATensor[...,R_samp,Theta_samp]
#         # polar_img = torch.reshape(polar_img,(RATensor.shape[0],RATensor.shape[1],out_pixels,out_pixels))
#         if torch.is_tensor(RATensor):
#             polar_img = torch.transpose(polar_img,-1,-2)
#         else:
#             polar_img = np.swapaxes(polar_img,-2,-1)
#     else:
#         polar_img = RATensor[R_samp,Theta_samp]
#         # polar_img = torch.reshape(polar_img,(out_pixels,out_pixels))
#         polar_img = polar_img.T

#     return polar_img