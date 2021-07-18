import math

from skyengine.geometry import haversine_distance

EARTH_RADIUS_KM = 6378.1


class Coordinate(object):
    """
    Describes a (latitude, longitude) coordinate and exposes some operations that can be performed
    on the coordinate.
    """

    def __init__(self, lat, lon):
        """
        Initialize a new Coordinate.

        :param lat: Latitude, as a float.
        :param lon: Longitude, as a float.
        """
        self.lat = lat
        self.lon = lon

    def pair(self):
        """
        Transform this coordinate definition into a Python tuple.

        :return: Tuple of (latitude, longitude).
        """
        return self.lat, self.lon

    def bearing_toward(self, target):
        """
        Calculate the azimuth bearing toward a target coordinate, in degrees.

        :param target: Another Coordinate instance toward which the bearing should be calculated.
        :return: Azimuth bearing value, in degrees.
        """
        # Shamelessly stolen from https://gis.stackexchange.com/questions/157693/
        start_lat = math.radians(self.lat)
        start_lng = math.radians(self.lon)
        end_lat = math.radians(target.lat)
        end_lng = math.radians(target.lon)
        d_long = end_lng - start_lng
        d_phi = math.log(math.tan(end_lat / 2.0 + math.pi / 4.0) /
                         math.tan(start_lat / 2.0 + math.pi / 4.0))
        if abs(d_long) > math.pi:
            if d_long > 0.0:
                d_long = -(2.0 * math.pi - d_long)
            else:
                d_long = (2.0 * math.pi + d_long)
        bearing = (math.degrees(math.atan2(d_long, d_phi)) + 360.0) % 360.0
        return bearing

    def offset_bearing(self, bearing, offset):
        """
        Calculate the new coordinate created by offsetting the current coordinate at a specified
        bearing, for a specified distance in meters.

        :param bearing: Azimuth bearing from this coordinate to the target coordinate, in degrees.
        :param offset: Number of meters to offset in the bearing direction.
        :return: Coordinate instance created from offsetting the current coordinate.
        """
        # Shamelessly stolen from https://gis.stackexchange.com/questions/157693/
        brng = math.radians(bearing)  # Bearing is degrees converted to radians.
        d = offset / 1000.0  # Distance m converted to km
        lat1 = math.radians(self.lat)  # Current dd lat point converted to radians
        lon1 = math.radians(self.lon)  # Current dd long point converted to radians
        lat2 = math.asin(math.sin(lat1) * math.cos(d / EARTH_RADIUS_KM) + math.cos(lat1) *
                         math.sin(d / EARTH_RADIUS_KM) * math.cos(brng)
                         )
        lon2 = lon1 + math.atan2(math.sin(brng) * math.sin(d / EARTH_RADIUS_KM) * math.cos(lat1),
                                 math.cos(d / EARTH_RADIUS_KM) - math.sin(lat1) * math.sin(lat2)
                                 )
        # convert back to degrees
        lat2 = math.degrees(lat2)
        lon2 = math.degrees(lon2)
        return Coordinate(lat2, lon2)

    def offset_toward_target(self, target, offset):
        """
        Calculate the new coordinate created by offsetting the current coordinate a specified number
        of meters toward the target coordinate.

        :param target: Target coordinate, represented as a Coordinate instance.
        :param offset: Number of meters to offset in the direction of the target coordinate.
        :return: Coordinate instance representing the destination location.
        """
        return self.offset_bearing(self.bearing_toward(target), offset)

    def distance_to(self, target):
        """
        Calculate the distance, in meters, toward another Coordinate instance.

        :param target: Coordinate instance describing another location.
        :return: Distance to the coordinate in meters.
        """
        return haversine_distance(self.pair(), target.pair())

    @staticmethod
    def from_tuple(pair):
        """
        Create a Coordinate from a Python tuple.

        :param pair: Tuple of (latitude, longitude).
        :return: Coordinate instance representing the specified location.
        """
        lat, lon = pair
        return Coordinate(lat, lon)

    @staticmethod
    def from_gps_data(gps_data):
        """
        Create a Coordinate from a Skyengine GPSData instance.

        :param gps_data: GPSData instance.
        :return: Coordinate instance representing the specified location.
        """
        return Coordinate(gps_data.lat, gps_data.lon)

    def __repr__(self):
        """
        String representation of a coordinate.

        :return: A string representation of (latitude, longitude).
        """
        return '({lat}, {lon})'.format(lat=self.lat, lon=self.lon)

