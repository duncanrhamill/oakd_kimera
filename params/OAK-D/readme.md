# OAK-D Parameters

These parameters were originally taken from the Euroc example in Kimera-VIO.
They have been modified for my OAK-D unit, you may have to change them for
yours.

## How to set these parameters

> Note: this may not actually be necessary as the images are already
> calibrated.

Grab the (depthai)[] repo and run 
```
python3 depthai_demo.py
```

You can CTRL-C once an image appears, what we want is the EEPROM data that is
printed out at the start, it should look something like this:

```
EEPROM data: valid (v5)
  Board name     : BW1098OBC
  Board rev      : R0M0E0
  HFOV L/R       : 71.86 deg
  HFOV RGB       : 68.7938 deg
  L-R   distance : 7.5 cm
  L-RGB distance : 3.75 cm
  L/R swapped    : yes
  L/R crop region: center
  Rectification Rotation R1 (left):
    0.999953,    0.006138,    0.007479,
   -0.006151,    0.999980,    0.001609,
   -0.007469,   -0.001654,    0.999971,
  Rectification Rotation R2 (right):
    0.999891,    0.014718,    0.000667,
   -0.014717,    0.999890,   -0.001636,
   -0.000691,    0.001626,    0.999998,
  Calibration intrinsic matrix M1 (left):
  859.042969,    0.000000,  639.301880,
    0.000000,  859.611694,  412.302124,
    0.000000,    0.000000,    1.000000,
  Calibration intrinsic matrix M2 (right):
  853.783875,    0.000000,  646.009705,
    0.000000,  853.774292,  407.493500,
    0.000000,    0.000000,    1.000000,
  Calibration rotation matrix R:
    0.999940,   -0.008578,    0.006763,
    0.008555,    0.999958,    0.003345,
   -0.006792,   -0.003287,    0.999972,
  Calibration translation matrix T:
   -7.462184,
   -0.109841,
   -0.004980,
  Calibration Distortion Coeff d1 (Left):
   -5.039202,   16.144112,    0.001593,    0.000017,  -16.302311,   -5.093875,   16.366404,
  -16.517309,    0.000000,    0.000000,    0.000000,    0.000000,    0.000000,    0.000000,
  Calibration Distortion Coeff d2 (Right):
   -4.910755,   15.082255,   -0.000957,   -0.001710,  -14.818521,   -4.963440,   15.288989,
  -15.017061,    0.000000,    0.000000,    0.000000,    0.000000,    0.000000,    0.000000,
```

Fill in this data to `calc_params.yaml`, then 
```shell
python3 calc_params.py
```

Copy this data into the parameter files.