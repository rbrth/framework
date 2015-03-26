3.1.3 Camera Calibration (source: https://github.com/tum-vision/lsd_slam)

LSD-SLAM operates on a pinhole camera model, however we give the option to undistort images before 
they are being used. You can find some sample calib files in lsd_slam_core/calib.
Calibration File for FOV camera model:

fx/width fy/height cx/width cy/height d
in_width in_height
"crop" / "full" / "none" / "e1 e2 e3 e4 0"
out_width out_height

Here, the values in the first line are the camera intrinsics and radial distortion parameter as 
given by the PTAM cameracalibrator, in_width and in_height is the input image size, and out_width
out_height is the desired undistorted image size. The latter can be chosen freely, however 640x480 
is recommended as explained in section 3.1.6. The third line specifies how the image is distorted, 
either by specifying a desired camera matrix in the same format as the first four intrinsic 
parameters, or by specifying "crop", which crops the image to maximal size while including 
only valid image pixels.

Calibration File for Pre-Rectified Images:

This one is without radial distortion correction, as a special case of ATAN camera model but without the computational cost:

fx/width fy/height cx/width cy/height 0
width height
none
width height


"The focal length fx (for example) is actually the product of the physical focal length of the lens 
and the size sx of the individual imager elements (this should make sense because sx has units of 
pixels per millimeter while F has units of millimeters, which means that fx is in the required units 
of pixels). It is important to keep in mind, though, that sx and sy cannot be measured directly 
via any camera calibration process, and neither is the physical focal length F directly measurable.
 Only the combinations fx = F*sx and fy = F*sy can be derived without actually dismantling the 
 camera and measuring its components directly."
