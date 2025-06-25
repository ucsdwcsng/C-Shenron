# New masking procedure for FBLR concatenation

import numpy as np

def convert_angle_degree_to_pixel(angle_degrees, in_pixels, angle = None):
    if angle == "radian":
        return (np.degrees(angle_degrees) / 180) * (in_pixels / 2) + in_pixels / 2
    return int((angle_degrees / 180) * (in_pixels / 2) + in_pixels / 2)

def cart2polar_for_mask(x, y, in_pixels):
    # Don't worry about range because all are going to be one
    r = np.sqrt(x ** 2 + y ** 2)
    
    # Assuming uniform theta resolution
    theta = np.arctan2(y, x)
    theta_px = convert_angle_degree_to_pixel(theta, in_pixels, angle = "radian")
    return r, theta_px

def generate_mask(shape, start_angle, fov_degrees, overlap_mag = 0.5, end_mag = 0.5):
    # Origin at the center
    X, Y = np.meshgrid(np.linspace(-shape / 2, shape / 2, shape), np.linspace(-shape / 2, shape / 2, shape))
    # Rotating the axes to make the plane point upwards
    X = np.rot90(X)
    Y = np.rot90(Y)
    
    R, theta = cart2polar_for_mask(X, Y, shape)
    
    R = R.astype(int)
    theta = theta.astype(int)
    
    mask_polar = np.zeros((shape, shape))
    
    a = convert_angle_degree_to_pixel(-start_angle, shape)
    b = convert_angle_degree_to_pixel(start_angle, shape)
    
    mask_polar[:, a : b] = 1
    
    fov_pixels_a = convert_angle_degree_to_pixel(-fov_degrees / 2, shape)
    fov_pixels_b = convert_angle_degree_to_pixel(fov_degrees / 2, shape)
    
    mask_polar[:, fov_pixels_a : a] = np.linspace(end_mag, 1, a - fov_pixels_a).reshape(1, a - fov_pixels_a)
    mask_polar[:, b : fov_pixels_b] = np.linspace(1, end_mag, fov_pixels_b - b).reshape(1, fov_pixels_b - b)
    
    mask_cartesian = mask_polar[R, theta]
    
    return mask_cartesian