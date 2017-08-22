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

# Identify pixels between the bounds
def color_thresh_between(img, low_thresh, high_thresh):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > low_thresh[0]) \
                & (img[:,:,0] < high_thresh[0]) \
                & (img[:,:,1] > low_thresh[1]) \
                & (img[:,:,1] < high_thresh[1]) \
                & (img[:,:,2] > low_thresh[2]) \
                & (img[:,:,2] < high_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
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
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
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

    mask = cv2.warpPerspective(img[:,:,0], M, (img.shape[1], img.shape[0]))

    return warped, mask

# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    image = Rover.img

    dst_size = 5
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[image.shape[1]/2 - dst_size, image.shape[0] - bottom_offset],
                      [image.shape[1]/2 + dst_size, image.shape[0] - bottom_offset],
                      [image.shape[1]/2 + dst_size, image.shape[0] - 2*dst_size - bottom_offset],
                      [image.shape[1]/2 - dst_size, image.shape[0] - 2*dst_size - bottom_offset],
                      ])

    warped, mask = perspect_transform(Rover.img, source, destination)

    navigable_area = color_thresh(warped, (160, 160, 160))
    obstacles = np.absolute(np.float32(navigable_area) - 1) * mask
    samples_area = color_thresh_between(warped, (0, 105, 0), (255, 220, 65))

    Rover.vision_image[:,:,0] = obstacles * 255
    Rover.vision_image[:,:,1] = samples_area * 255
    Rover.vision_image[:,:,2] = navigable_area * 255

    xpix, ypix = rover_coords(navigable_area)
    x_world, y_world = pix_to_world(xpix, ypix, Rover.pos[0], Rover.pos[1], Rover.yaw, Rover.worldmap.shape[0], 10)

    xpix_obstacles, ypix_obstacles = rover_coords(obstacles)
    x_world_obstacles, y_world_obstacles = pix_to_world(xpix_obstacles, ypix_obstacles, Rover.pos[0], Rover.pos[1], Rover.yaw, Rover.worldmap.shape[0], 10)

    xpix_samples, ypix_samples = rover_coords(samples_area)
    x_world_samples, y_world_samples = pix_to_world(xpix_samples, ypix_samples, Rover.pos[0], Rover.pos[1], Rover.yaw, Rover.worldmap.shape[0], 10)

    dist, angles = to_polar_coords(xpix, ypix)
    dist_obstacles, angles_obstacles = to_polar_coords(xpix_obstacles, ypix_obstacles)
    dist_samples, angles_samples = to_polar_coords(xpix_samples, ypix_samples)

    if (Rover.pitch < 0.5 or Rover.pitch > 359.5) and (Rover.roll > 359 or Rover.roll < 1):
        Rover.worldmap[y_world_samples, x_world_samples, 1] += 1
        if not (np.count_nonzero(angles_samples > -0.1) > 0):
            Rover.worldmap[y_world_obstacles, x_world_obstacles, 0] += 1
            Rover.worldmap[y_world, x_world, 0] -= 5
            Rover.worldmap[y_world, x_world, 2] += 10

    if np.count_nonzero(angles_samples > -0.1) > 0:
        Rover.stop_forward = 1
        Rover.throttle_set = 0.75
        Rover.max_vel = np.mean(dist_samples[angles_samples > -0.1]) * (4 / 90)
        Rover.nav_dists = dist_obstacles + 5
        Rover.nav_angles = angles_samples[angles_samples > -0.1]
    else:
        Rover.stop_forward = 50
        Rover.throttle_set = 0.75
        Rover.nav_dists = dist_obstacles

        if (np.count_nonzero(angles[angles > -0.5]) > 20):
            angles_filter = angles > -0.5
        elif (np.count_nonzero(angles[angles > -1]) > 20):
            angles_filter = angles > -1
        else:
            angles_filter = angles != ''

        if (np.count_nonzero(angles_filter) > 0):
            Rover.max_vel = np.mean(dist) * (4 / 90)
            Rover.nav_angles = angles[angles_filter]
        else:
            Rover.max_vel = np.mean(dist) * (4 / 90)
            Rover.nav_angles = angles

        if (Rover.pitch > 350):
            Rover.max_vel += 1

    return Rover
