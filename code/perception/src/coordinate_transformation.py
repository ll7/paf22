"""
This script provides coordinate transformations from Geodetic -> ECEF,
ECEF -> ENU and Geodetic -> ENU
(the composition of the two previous functions).
Running the script by itself runs tests.
based on https://gist.github.com/govert/1b373696c9a27ff4c72a.
A good source to read up on the different reference frames is:
http://dirsig.cis.rit.edu/docs/new/coordinates.html
"""
import math
from enum import Enum


# Class to choose a map with a predefined reference point
class GeoRef(Enum):  # todo: add values for other towns
    TOWN01 = 0, 0, 0  # lat = 1.7e-08, lon = -5.6e-08, alt = 0.02
    TOWN02 = 0, 0, 0  # lat = 6.7e-10, lon= -3.4e-11, alt = -0.004
    TOWN03 = 0, 0, 0  # lat = 5.1e-10, lon = 2.1e-10, alt = 0.03
    TOWN04 = 0, 0, 0  # 0,0,0 not possible, but ref is correct
    TOWN05 = 0, 0, 0  # lat =2.6e-09, lon =8.7e-11, alt =-0.004 #fav
    TOWN06 = 0, 0, 0  # lat =, lon =, alt = #Town06/HD not found
    TOWN07 = 0, 0, 0  # lat =, lon =, alt = #Town07/HD not found
    TOWN08 = 0, 0, 0  # lat =, lon =, alt = #Town08/HD not found
    TOWN09 = 0, 0, 0  # lat =, lon =, alt = #Town09/HD not found
    TOWN10 = 0, 0, 0  # lat =-8.9e-05, lon =-3.1e-11, alt = 0.0 #Town10HD
    TOWN11 = 0, 0, 0  # lat =, lon =, alt = #Town11/HD not found
    TOWN12 = 35.25000, -101.87500, 331.00000


a = 6378137
b = 6356752.3142
f = (a - b) / a
e_sq = f * (2 - f)


class CoordinateTransformer:
    """Object that is used to transform Coordinates between
    xyz and gnss reference frame"""

    la_ref: float
    ln_ref: float
    h_ref: float
    ref_set = False

    def __init__(self):
        self.la_ref = 0
        self.ln_ref = 0
        self.h_ref = 0

    def set_gnss_ref(self, lat0, lon0, h0):
        """
        Set the given parameters as a reference point for x,y,z = 0,0,0
        :param lat0: Lateral zero point reference
        :param lon0: Longitudinal zero point reference
        :param h0: Height zero point reference
        :return: Nothing
        """
        self.la_ref = lat0
        self.ln_ref = lon0
        self.h_ref = h0
        self.ref_set = True

    def gnss_to_xyz(self, lat, lon, h):
        if self.ref_set:
            return geodetic_to_enu(lat, lon, h,
                                   self.la_ref, self.ln_ref, self.h_ref)
        else:
            return -1, -1, -1


def geodetic_to_enu(lat, lon, h, lat_ref, lon_ref, h_ref):
    x, y, z = geodetic_to_ecef(lat, lon, h)
    return ecef_to_enu(x, y, z, lat_ref, lon_ref, h_ref)


def geodetic_to_ecef(lat, lon, h):
    # (lat, lon) in WSG-84 degrees
    # h in meters
    lamb = math.radians(lat)
    phi = math.radians(lon)
    s = math.sin(lamb)
    N = a / math.sqrt(1 - e_sq * s * s)

    sin_lambda = math.sin(lamb)
    cos_lambda = math.cos(lamb)
    sin_phi = math.sin(phi)
    cos_phi = math.cos(phi)

    x = (h + N) * cos_lambda * cos_phi
    y = (h + N) * cos_lambda * sin_phi
    z = (h + (1 - e_sq) * N) * sin_lambda

    return x, y, z


def ecef_to_enu(x, y, z, lat0, lon0, h0):
    lamb = math.radians(lat0)
    phi = math.radians(lon0)
    s = math.sin(lamb)
    N = a / math.sqrt(1 - e_sq * s * s)

    sin_lambda = math.sin(lamb)
    cos_lambda = math.cos(lamb)
    s_phi = math.sin(phi)
    c_phi = math.cos(phi)

    x0 = (h0 + N) * cos_lambda * c_phi
    y0 = (h0 + N) * cos_lambda * s_phi
    z0 = (h0 + (1 - e_sq) * N) * sin_lambda

    xd = x - x0
    yd = y - y0
    zd = z - z0

    xE = -s_phi * xd + c_phi * yd
    yN = -c_phi * sin_lambda * xd - sin_lambda * s_phi * yd + cos_lambda * zd
    zUp = cos_lambda * c_phi * xd + cos_lambda * s_phi * yd + sin_lambda * zd

    return xE, yN, zUp

#
# if __name__ == '__main__':
#    def are_close(a, b):
#        return abs(a - b) < 1e-4
#
#
#    latLA = 34.00000048
#    lonLA = -117.3335693
#    hLA = 251.702
#
#    x0, y0, z0 = geodetic_to_ecef(latLA, lonLA, hLA)
#    x = x0 + 1
#    y = y0
#    z = z0
#    xEast, yNorth, zUp = ecef_to_enu(x, y, z, latLA, lonLA, hLA)
#    assert are_close(0.88834836, xEast)
#    assert are_close(0.25676467, yNorth)
#    assert are_close(-0.38066927, zUp)
#
#    x = x0
#    y = y0 + 1
#    z = z0
#    xEast, yNorth, zUp = ecef_to_enu(x, y, z, latLA, lonLA, hLA)
#    assert are_close(-0.45917011, xEast)
#    assert are_close(0.49675810, yNorth)
#    assert are_close(-0.73647416, zUp)
#
#    x = x0
#    y = y0
#    z = z0 + 1
#    xEast, yNorth, zUp = ecef_to_enu(x, y, z, latLA, lonLA, hLA)
#    assert are_close(0.00000000, xEast)
#    assert are_close(0.82903757, yNorth)
#    assert are_close(0.55919291, zUp)
