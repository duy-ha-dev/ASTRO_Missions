import math

from skyengine.geometry import haversine_distance

EARTH_RADIUS_KM = 6378.1


class Waypoint(object):
    """
    Describes a waypoint with latitude, longitude, and height that the drone can fly to with goto
    """

    def __init__(self, lat, lon, alt):
        """
        Initialize a new Waypoint.

        :param lat: Latitude, as a float.
        :param lon: Longitude, as a float.
        :param alt: Altitude, as a float.
        """
        self.lat = lat
        self.lon = lon
        self.alt = alt

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

    def serialize(self):
        return {
            'lat': self.lat,
            'lon': self.lon,
        }

    @staticmethod
    def deserialize(json):
        return Coordinate(
            lat=json['lat'],
            lon=json['lon'],
        )

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

    @staticmethod
    def from_waypoint(waypoint):
        """
        Create a Coordinate from a Waypoint instance

        :param waypoint: Waypoint instance
        :return: Coordinate instance representing the specified location.
        """
        return Coordinate(waypoint.lat, waypoint.lon)

    def __repr__(self):
        """
        String representation of a coordinate.

        :return: A string representation of (latitude, longitude).
        """
        return '({lat}, {lon})'.format(lat=self.lat, lon=self.lon)


class Region(object):
    """
    Describes a quadrilateral region enclosed by four coordinates.
    """

    def __init__(self, nw, ne, sw, se):
        """
        Initialize a new Region.

        :param nw: Northwest coordinate (top left), as a Coordinate instance.
        :param ne: Northeast coordinate (top right), as a Coordinate instance.
        :param sw: Southwest coordinate (bottom left), as a Coordinate instance.
        :param se: Southeast coordinate (bottom right), as a Coordinate instance.
        """
        self.nw = nw  # Top left
        self.ne = ne  # Top right
        self.sw = sw  # Bottom left
        self.se = se  # Bottom right


    def corners(self):
        """
        Returns a list of coordinate tuples for the corners of the Regiom
        """
        return [self.nw.pair(),
                self.ne.pair(), 
                self.sw.pair(), 
                self.se.pair()]

    def perimeter_path(self):
        """
        Returns a list of Coordinate instances of the corners of the Region
        in a counter-clockwise direction
        """
        # They are now in a different order than they are given when the Region instance is created
        return [self.ne,
                self.nw,
                self.sw,
                self.se]

    def survey_coordinates(self, current_location_coord):
        """
        Generate coordinates used to fully survey a specified region.

        :param current_location_coord: a Coordinate of the drone's current location
        :return: A list of Coordinate objects representing the locations to visit, in order, to perform
                 a full survey of the region.
        """
        corner_coords = [self.nw, self.ne, self.sw, self.se]
        dist = [corner_coords[-1].distance_to(corner_coords[i]) for i in range(3)]

        # use the distances in order to pair the corner indeces with the neighbors
        dist_copy = dist[:]
        dist_copy.sort()
        neighbor_close = dist.index(min(dist))
        neighbor_far = dist.index(dist_copy[1])
        neighbor_diag = dist.index(dist_copy[2])
        sides = {}
        sides['short'] = {3: neighbor_close, neighbor_close: 3, neighbor_far: neighbor_diag, neighbor_diag: neighbor_far}
        sides['long'] = {3: neighbor_far, neighbor_far: 3, neighbor_close: neighbor_diag, neighbor_diag: neighbor_close}

        # find closest corner to drone
        calc_dist = [current_location_coord.distance_to(corner) for corner in corner_coords]
        closest_corner = calc_dist.index(min(calc_dist))
        second_corner = sides['long'][closest_corner]
        closest_corner_coord = corner_coords[closest_corner]
        second_corner_coord = corner_coords[second_corner]

        # make the coordinates for the traversal points of rotation
        traverse_coords = [closest_corner_coord, second_corner_coord]
        shortest_side_dist = min([closest_corner_coord.distance_to(corner_coords[sides['short'][closest_corner]]),
                                  second_corner_coord.distance_to(corner_coords[sides['short'][second_corner]])])
        num_of_sweeps = int(round(shortest_side_dist / 10))
        if num_of_sweeps != 0:
            dist_sweep = shortest_side_dist / float(num_of_sweeps)
        else:
            dist_sweep = 0

        for i in range(num_of_sweeps):
            if (i % 2) == 0 :
                new_coord1 = traverse_coords[-1].offset_toward_target(corner_coords[sides['short'][second_corner]], dist_sweep)
                new_coord2 = traverse_coords[-2].offset_toward_target(corner_coords[sides['short'][closest_corner]], dist_sweep)
            else:
                new_coord2 = traverse_coords[-2].offset_toward_target(corner_coords[sides['short'][second_corner]], dist_sweep)
                new_coord1 = traverse_coords[-1].offset_toward_target(corner_coords[sides['short'][closest_corner]], dist_sweep)
            traverse_coords.extend([new_coord1,new_coord2])

        return traverse_coords


    def survey_waypoints(self, current_location_coord, altitudes):
        """
        Generate waypoints used to fully survey a specified region at the given altitudes

        :param current_location_coord: a Coordinate of the drone's current location
        :return: A list of Waypoint objects representing the locations to visit, in order, to perform
                 a full survey of the region.
        """
        
        # use the coordinates to make waypoints at all 3 altitudes
        traverse_coords = self.survey_coordinates(current_location_coord)

        waypoints = []
        for i, alt in enumerate(altitudes):
            if (i%2) == 0:
                for coord in traverse_coords:
                    waypoints.append(coord_to_waypoint(coord, alt))
            else:
                for coord in reversed(traverse_coords):
                    waypoints.append(coord_to_waypoint(coord, alt))

        return waypoints


    def vertical_partition(self, num_partitions):
        """
        Vertically partition the region (i.e., along longitude lines) into the specified number of
        partitions.

        :param num_partitions: Number of sub-regions to generate from this region.
        :return: An array of Region instances describing partitions of the original region.
        """
        top_lat_offset = (self.ne.lat - self.nw.lat) / num_partitions
        top_lon_offset = (self.ne.lon - self.nw.lon) / num_partitions
        bottom_lat_offset = (self.se.lat - self.sw.lat) / num_partitions
        bottom_lon_offset = (self.se.lon - self.sw.lon) / num_partitions

        return [
            Region(
                nw=Coordinate(
                    lat=self.nw.lat + part * top_lat_offset,
                    lon=self.nw.lon + part * top_lon_offset,
                ),
                ne=Coordinate(
                    lat=self.nw.lat + (part + 1) * top_lat_offset,
                    lon=self.nw.lon + (part + 1) * top_lon_offset,
                ),
                sw=Coordinate(
                    lat=self.sw.lat + part * bottom_lat_offset,
                    lon=self.sw.lon + part * bottom_lon_offset,
                ),
                se=Coordinate(
                    lat=self.sw.lat + (part + 1) * bottom_lat_offset,
                    lon=self.sw.lon + (part + 1) * bottom_lon_offset,
                ),
            )
            for part in xrange(num_partitions)
        ]

    def center(self):
        """
        Calculate the approximate center coordinate of the region.

        :return: A Coordinate instance describing the approximate center of the region.
        """
        return Coordinate(
            lat=(self.nw.lat + self.ne.lat + self.sw.lat + self.se.lat) / 4.0,
            lon=(self.nw.lon + self.ne.lon + self.sw.lon + self.se.lon) / 4.0,
        )

    def shrink(self, distance):
        """
        Create a new region created by shrinking the corners of the current region by a specified
        distance.

        :param distance: Number of meters that each corner of the region should be "pulled" inwards.
        :return: A new Region instance describing a shrunken region.
        """
        return Region(
            nw=self.nw.offset_toward_target(self.se, distance),
            ne=self.ne.offset_toward_target(self.sw, distance),
            sw=self.sw.offset_toward_target(self.ne, distance),
            se=self.se.offset_toward_target(self.nw, distance),
        )

    def internal_distance(self):
        """
        Calculate the smallest internal distance within the region (e.g., the minimum of the
        two distances between the NW-SE and NE-SW paths).

        :return: A float describing the region's internal traversal distance, in meters.
        """
        return min([
            self.nw.distance_to(self.se),
            self.ne.distance_to(self.sw),
        ])

    def __repr__(self):
        """
        String representation of a region.

        :return: A string representation of the four bounding corners of the region.
        """
        return 'Region(nw={nw}, ne={ne}, sw={sw}, se={se})'.format(
            nw=self.nw,
            ne=self.ne,
            sw=self.sw,
            se=self.se,
        )
