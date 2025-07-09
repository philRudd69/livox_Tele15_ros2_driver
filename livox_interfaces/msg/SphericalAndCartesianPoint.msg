# Livox costom pointcloud format.

uint32 offset_time      # offset time relative to the base time
float32 x               # X axis, unit:m
float32 y               # Y axis, unit:m
float32 z               # Z axis, unit:m
float32 theta           # Azimuth, horizontal angle in xy-plane, measured from positive x-axis counter-clockwise, Unit:rad
float32 phi             # Elevation, vertical angle in yz-plane, measured from xy-plane upwards as positive, Unit:rad
float32 r               # Range, radial distance, Unit:m
uint8 reflectivity      # reflectivity, 0~255
uint8 tag               # livox tag
uint8 line              # laser number in lidar
