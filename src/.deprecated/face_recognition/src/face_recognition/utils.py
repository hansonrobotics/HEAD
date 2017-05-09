import math
def get_3d_point(bbox, cam_width=640, cam_height=480, cam_fov=0.625):
    # TODO will need to be updated:
    # Current camera calibration matrix should be passed.
    # Current camera pose needed (offset, and angle)
    # For now we assume its 36 degrees FOV with the face height of 20 cm
    # Standard 640x480 image used (taken from config)
    # Approx horizontal FOV of camera used (taken from config)
    # FOV-X and FOV-Y are different, now using FOV-X
    bbox_size = bbox.width()*bbox.height()
    if bbox.width()*bbox.height() == 0:
        logger.warn("Bounding box is of 0 size")
        return None

    K_const = cam_width / math.tan(cam_fov/2.0)
    dp = 0.17 / float(bbox_size) # It should be same in both axis
    # Y is to the left in camera image, Z is to top
    x = dp * K_const
    y = dp * (cam_width-(bbox.left()+bbox.right())/2)
    z = dp * (cam_height-(bbox.top()+bbox.bottom())/2)
    return x, y, z
