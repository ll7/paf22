import numpy as np
from coordinate_transformation import CoordinateTransformer

transformer = CoordinateTransformer()
transformer.__init__()

# Defining points for testing
lat = np.zeros(7)
lon = np.zeros(7)

# Aus der falschen Fahrrichtung kommend
lat[0], lon[0] = 35.202054041307136, -101.86416901902722
lat[1], lon[1] = 35.20189159192131, -101.86417409137256
lat[2], lon[2] = 35.20182086689459, -101.86418153253398

# "Ursprung"
lat[3], lon[3] = 35.20171220851982, -101.86417900962616

# In Fahrrichtung unterwegs
lat[4], lon[4] = 35.20171306292836, -101.86417678477729
lat[5], lon[5] = 35.201512787762624, -101.86419119550204
lat[6], lon[6] = 0, 0


# Setting reference
gps_lat_ref, gps_lon_ref = 35.25000, -101.87500
m_h_ref = 331.7265
transformer.set_gnss_ref(gps_lat_ref, gps_lon_ref, m_h_ref)

# print(transformer.gnss_to_xyz(lat[6], lon[6], 373.01))

for i in range(0, 7):
    gps1_lat = lat[i]
    gps1_lon = lon[i]
    gps1_h = 373.112
    x, y, z = transformer.gnss_to_xyz(gps1_lat, gps1_lon, gps1_h)
    print(x, y, z)
