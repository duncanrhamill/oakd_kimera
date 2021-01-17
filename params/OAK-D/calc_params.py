import copy
import cv2
import numpy as np
from pprint import pprint

## INPUT DATA

# Calibration intrinsic matrix M1 (left):
left_intrinsic = np.array([
    [859.042969,    0.000000,  639.301880],
    [0.000000,    859.611694,  412.302124],
    [0.000000,      0.000000,    1.000000]
])

# Calibration intrinsic matrix M2 (right):
right_intrinsic = np.array([
    [853.783875,    0.000000,  646.009705],
    [0.000000,    853.774292,  407.493500],
    [0.000000,      0.000000,    1.000000]
])

# Calibration rotation matrix R:
calib_rotation = np.array([
    [ 0.999940,   -0.008578,    0.006763],
    [ 0.008555,    0.999958,    0.003345],
    [-0.006792,   -0.003287,    0.999972]
])

# Calibration translation matrix T: (this should be in cm, don't change it from
# what's output by depthai_demo.py!)
calib_translation = np.array([
    [-7.462184],
    [-0.109841],
    [-0.004980]
])

# Calibration Distortion Coeff d1 (Left):
calib_distortion_left = np.array([
    -5.039202,   16.144112,    0.001593,    0.000017,  -16.302311,   -5.093875,   16.366404,
    -16.517309,    0.000000,    0.000000,    0.000000,    0.000000,    0.000000,    0.000000,
])

# Calibration Distortion Coeff d2 (Right):
calib_distortion_right = np.array([
   -4.910755,   15.082255,   -0.000957,   -0.001710,  -14.818521,   -4.963440,   15.288989,
   -15.017061,    0.000000,    0.000000,    0.000000,    0.000000,    0.000000,    0.000000,
])

## CONSTANTS

width = 1280
height = 720
framerate = 20
camera_model = 'pinhole'
distortion_model = 'radial-tangential'

## VALUES

# Left camera is our body frame. Extrinsic matrices are in the format
# [ R_3x3 T_3x1; 0 0 0 1 ], so a unit rotation matrix and empty translation
# vector will mean that our left camera matrix is just a 4x4 identiy matrix.
left_extrinsic = np.eye(4)

# For the right extrinsic we calculate our matrix based on the calibration data
# provided above, remembering that the translation vetor is actually in
# centimeters. 
right_extrinsic = np.vstack((
    np.hstack((calib_rotation, calib_translation * 0.01)),
    [0., 0., 0., 1.]
))

# Extract the intrinsics vectors from the intrinsic matrices
left_intrinsic_vector = np.array([left_intrinsic[0][0], left_intrinsic[1][1], left_intrinsic[0][2], left_intrinsic[1][2]])
right_intrinsic_vector = np.array([right_intrinsic[0][0], right_intrinsic[1][1], right_intrinsic[0][2], right_intrinsic[1][2]])

# Calculate the rotation and projection matrices
R1, R2, P1, P2, Q, ROI1, ROI2 = cv2.stereoRectify(
    left_intrinsic,
    calib_distortion_left,
    right_intrinsic,
    calib_distortion_right,
    (width, height),
    calib_rotation,
    calib_translation
)

pprint(R1)
pprint(R2)
pprint(P1)
pprint(P2)

## BUILD CALIBRATION DICTS

calib_left = {
    'camera_id': 'left_cam',
    'T_BS': {
        'cols': 4,
        'rows': 4,
        'data': left_extrinsic
    },
    'rate_hz': 20,
    'resolution': [width, height],
    'camera_model': camera_model,
    'intrinsics': left_intrinsic_vector,
    'distortion_model': distortion_model,
    'distortion_coefficients': calib_distortion_left[0:4]
}

calib_right = copy.deepcopy(calib_left)
calib_right['T_BS']['data'] = right_extrinsic
calib_right['intrinsics'] = right_intrinsic_vector
calib_right['distortion_coefficients'] = calib_distortion_right[0:4]

## WRITE CALIB FILES

# TODO: Can't use pyyaml because it expects OpenCV formatted yaml, which is
# different. Write something to print these out in OpenCV format.

pprint(calib_left)
pprint(calib_right)