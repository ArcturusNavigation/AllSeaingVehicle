#---------- PERCEPTION ----------#

# Zed image size
IMG_HEIGHT = 360
IMG_WIDTH = 640

# Units for HD 720 Video feed (in degrees)
ZED_FOV = 107

# Buoy classes for object detection
BUOY_CLASSES = {
    "EAST": 0,
    "GREEN": 1,
    "RED": 2,
    "WEST": 3,
}

#---------- NAVIGATION ----------#

# PWM limits
PWM_MIN = 1100
PWM_MID = 1500
PWM_MAX = 1900

# Thruster mappings
THR_FL = 0
THR_FR = 0
THR_BL = 2
THR_BR = 3
