import math


_EARTH_RADIUS_METERS = 6.371e6


def haversine(a):
    """
    The 'Haversine' function. For a in radians.
    """
    # Min inserted to prevent floating-point errors.
    return min(1.0, math.sin(a / 2.0)**2)


def _deg2rad(d):
    return (math.pi * d / 180.0) % (2.0 * math.pi)


def haversine_distance(x, y):
    """
    The Haversine Formula calculates the "great circle" distance between two lat/lon pairs.
    :param x: a (lat, lon) or (lat, lon, elevation) tuple.
    :param y: a (lat, lon) or (lat, lon, elevation) tuple.
    :return: The "great circle" distance between the points.
    """
    phi_1, tht_1 = map(_deg2rad, x[:2])
    phi_2, tht_2 = map(_deg2rad, y[:2])
    d = 2 * _EARTH_RADIUS_METERS * math.asin(math.sqrt(
        haversine(phi_2 - phi_1) +
        math.cos(phi_1) * math.cos(phi_2) * haversine(tht_2 - tht_1)
        ))
    return d


def offset2latlon(meters_north, meters_east, current_lat):
    """
    Convert a position offset in meters to a relative latitude / longitude.
    :param meters_north: The number of meters to move northward.
    :param meters_east: The number of meters to move eastward.
    :param current_lat: The current latitude.
    """
    d_lat = (float(meters_north) / _EARTH_RADIUS_METERS) * 180.0 / math.pi
    d_lon = math.asin(math.sin(meters_east / (2 * _EARTH_RADIUS_METERS)) /
                      math.cos(current_lat)) * 180.0 / math.pi
    return (d_lat, d_lon)


def airborne_haversine_distance(pos_1, pos_2):
    """
    From two (lat, lon, alt) tuples in the same reference frame, compute the distance between them.
    Technically, this is slightly inaccurate -- it assumes that:
    1. The Earth is spherical.
    2. The distance between two points is well-approximated by a triangle whose base is the great
    circle distance between the points and whose height is the change in altitude. Provided the
    accuracy of the GPS and the distances that our drone can travel, these assumptions hold.
    However, if we add a "Launch ICBM" feature, this will need to be fixed.
    :param pos_1: A (lat, lon, alt) triple.
    :param pos_2: A (lat, lon, alt) triple in the same reference frame.
    :return: A good approximation of the distance between the two points.
    """
    ground_dist = haversine_distance(pos_1, pos_2)
    _, _, alt_1 = pos_1
    _, _, alt_2 = pos_2
    change_in_alt = alt_2 - alt_1
    return math.sqrt(ground_dist**2 + change_in_alt**2)
