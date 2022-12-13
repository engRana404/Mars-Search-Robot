import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

def find_obstacle(img, obs_thresh=(160, 160, 160)):
    color_select = np.zeros_like(img[:,:,0])
    obs = (img[:,:,0] < obs_thresh[0]) \
    & (img[:,:,1] < obs_thresh[1]) \
    & (img[:,:,2] < obs_thresh[2])
    color_select[obs] = 1
    return color_select

def find_rock(img, yellow_thresh=(100, 100, 20)):
    color_select = np.zeros_like(img[:,:,0])
    rock = (img[:,:,0] > yellow_thresh[0]) & (img[:,:,1] > yellow_thresh[1]) & (img[:,:,2] < yellow_thresh[2])
    color_select[rock] = 1
    return color_select

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel*2 + y_pixel*2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):   
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image    
    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    
    # 1) Define source and destination points for perspective transform
     # Dimension of source image from rover camera
    height, width = Rover.img.shape[0], Rover.img.shape[1]

    # Numpy array of four source points defining a grid on input 3D image
    # acquired from calibration data in test notebook
    src_x1, src_y1 = 14, 140
    src_x2, src_y2 = 301, 140
    src_x3, src_y3 = 200, 96
    src_x4, src_y4 = 118, 96
    bottom_offset = 5
    dst_grid = 6
    # Corresponding destination points on output 2D overhead image
    dst_x1, dst_y1 = (width/2 - dst_grid/2), (height-bottom_offset)
    dst_x2, dst_y2 = (width/2 + dst_grid/2), (height-bottom_offset)
    dst_x3, dst_y3 = (width/2 + dst_grid/2), (height-dst_grid-bottom_offset)
    dst_x4, dst_y4 = (width/2 - dst_grid/2), (height-dst_grid-bottom_offset)

    src = np.float32([[src_x1, src_y1],
                                [src_x2, src_y2],
                                [src_x3, src_y3],
                                [src_x4, src_y4]])

    dst = np.float32([[dst_x1, dst_y1],
                                [dst_x2, dst_y2],
                                [dst_x3, dst_y3],
                                [dst_x4, dst_y4]])  
    
    # 2) Apply perspective transform
    warped_img = perspect_transform(Rover.img, src, dst)
    rocks = perspect_transform(find_rock(Rover.img), src, dst)
    obstacles = perspect_transform(find_obstacle(Rover.img), src, dst)
    
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    thresh_pixpts_pf = color_thresh(warped_img)
    
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    Rover.vision_image[:,:,2] = thresh_pixpts_pf * 175
    Rover.vision_image[:,:,0] = obstacles * 135
    Rover.vision_image[:,:,1] = rocks * 1
    
    # 5) Convert map image pixel values to rover-centric coords
    x_pixpts_rf, y_pixpts_rf  = rover_coords(thresh_pixpts_pf)
    oxpix, oypix = rover_coords(obstacles)
    rxpix, rypix = rover_coords(rocks)    
    
    dist, angles= to_polar_coords(x_pixpts_rf, y_pixpts_rf)
    
    # 6) Convert rover-centric pixel values to world coordinates
    #worldmap = np.zeros((200, 200))
    worldmap = Rover.worldmap
    scale = 10  # scale factor assumed between world and rover space pixels
    obstacle_x_world, obstacle_y_world = pix_to_world(oxpix,oypix,Rover.pos[0],Rover.pos[1],Rover.yaw,worldmap.shape[0],scale)
    rock_x_world, rock_y_world = pix_to_world(rxpix,rypix,Rover.pos[0],Rover.pos[1],Rover.yaw,worldmap.shape[0],scale)    
    x_world, y_world = pix_to_world(x_pixpts_rf, y_pixpts_rf, Rover.pos[0],Rover.pos[1],Rover.yaw,worldmap.shape[0],scale)
    
    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
        
    if ((Rover.pitch < 1 or Rover.pitch > 359) and (Rover.roll < 1 or Rover.roll > 359)):
        Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        Rover.worldmap[y_world, x_world, 0] = 0
        Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        Rover.worldmap[y_world,x_world, 2] += 1           
    
    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    Rover.nav_dists = dist 
    Rover.nav_angles = angles 
    
    return Rover