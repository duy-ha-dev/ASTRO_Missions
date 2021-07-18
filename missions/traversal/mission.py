import argparse
import time

from skyengine.drone import DroneController
from skyengine.exceptions import FlightAbortedException
from skylog.logger import DirectLogger
from skylog.message import BaseMessage
from skymission.concurrency import tick
from skymission.mission import Mission
from skymission.mission import callback
from skymission.mission import panic

from geo import Coordinate


class Waypoint:
    def __init__(self, lat, lon, alt):
        self.lat = lat
        self.lon = lon
        self.alt = alt


class TraversalMessage(BaseMessage):
    """
    Message for the drone's current location, speed, and heading direction.
    """

    def __init__(self, timestamp, location, speed, heading):
        self.timestamp = timestamp
        self.lat = location.lat
        self.lon = location.lon
        self.alt = location.alt
        self.speed = speed
        self.heading = heading

    def serialize(self):
        return {
            'timestamp': self.timestamp,
            'location': {
                'lat': self.lat,
                'lon': self.lon,
                'alt': self.alt
            },
            'speed':self.speed,
            'heading':self.heading,
        }


def coord_to_waypoint(coordinate, alt):
    """
    Takes an instance of Coordinate and its altitude and outputs an instance of Waypoint
    """
    return Waypoint(coordinate.lat, coordinate.lon, alt)


class TraversalMission(Mission):
    """
    This mission will sweep the area of a designated region. It will determine the closet corner and
    begin there, then sweeping back and forth long ways along the region, from all inputted altitudes.
    """

    mission_id = 'traversal-mission'
    port = 4004

    def __init__(self, fc_addr, log_file):
        """
        Create a TraversalMission and start the mission server.

        :param fc_addr: MAVProxy address of the flight controller.
        :param log_file: Name of the log file for location data.
        """
        self.enable_disk_event_logging()

        self.dc = DroneController(fc_addr)
        self.logger = DirectLogger(path=log_file)
        self.log.debug('Drone controller and logger initialized successfully')

        self.start_server()


    @tick(interval=1)
    def start_log(self):
        """
        Start periodically logging the drone GPS location, speed, and direction.
        """
        gps_pos = self.dc.read_gps()
        self.logger.log(TraversalMessage(
            timestamp=time.time(),
            location=gps_pos,
            speed=self.dc.vehicle.airspeed,
            heading=self.dc.vehicle.heading,
        ))


    @callback(
        endpoint='/start-mission',
        description='Start the traversal mission: sweep the region starting from '
                    'nearest corner while reporting location, speed, and heading.',
        required_params=('corners', 'alt'),
        public=True,
    )
    def start_mission(self, data, *args, **kwargs):
        """
        Client invoked endpoint to begin the traversal mission

        :param data: Required to be of the form:
                     {
                         'corners': ..., #Name of set of corners matching desired region
                         'alt': [...] #List of altitudes in meters
                     }
        """
        altitudes = data['alt']
        corners = data['corners']
        corner_global = corners_names[corners]
    
        # make the corner lat/lon points into instances of Coordinate and compute the distances from 
        # each of them to the last one
        corner_coords = [Coordinate(c[0],c[1]) for c in corner_global]
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
        self.log.debug('Short side pairings = (3,{q}), ({w},{e})'.format(
            q=neighbor_close,
            w=neighbor_far,
            e=neighbor_diag,
        ))

        # offset all the corners to be 10 meters inward from each direction
        for side in sides:
            for a in sides[side]:
                b = sides[side][a]
                corner_coords[a] = corner_coords[a].offset_toward_target(corner_coords[b], 10)

        # find closest corner to drone
        current_location = self.dc.read_gps()
        current_location_coord = Coordinate(current_location.lat, current_location.lon)
        calc_dist = [current_location_coord.distance_to(corner) for corner in corner_coords]
        closest_corner = calc_dist.index(min(calc_dist))
        second_corner = sides['long'][closest_corner]
        closest_corner_coord = corner_coords[closest_corner]
        second_corner_coord = corner_coords[second_corner]

        self.log.debug('Current location is ({a},{b}) and closest corner is ({c},{d})'.format(
            a=current_location.lat, 
            b=current_location.lon,
            c=closest_corner_coord.lat,
            d=closest_corner_coord.lon,
            ))
    
        # calculate the coordinates for the traversal
        traverse_coords = [closest_corner_coord, second_corner_coord]
        shortest_side_dist = min([closest_corner_coord.distance_to(corner_coords[sides['short'][closest_corner]]),
                                  second_corner_coord.distance_to(corner_coords[sides['short'][second_corner]])])
        num_of_sweeps = int(round(shortest_side_dist / 10))
        if num_of_sweeps != 0:
            dist_sweep = shortest_side_dist / float(num_of_sweeps)
        else:
            dist_sweep = 0
        self.log.debug('Number of sweeps: {a} Distance per sweep: {b}'.format(a=num_of_sweeps,b=dist_sweep))
        
        for i in range(num_of_sweeps):
            if (i % 2) == 0 :
                new_coord1 = traverse_coords[-1].offset_toward_target(corner_coords[sides['short'][second_corner]], dist_sweep)
                new_coord2 = traverse_coords[-2].offset_toward_target(corner_coords[sides['short'][closest_corner]], dist_sweep)
            else:
                new_coord2 = traverse_coords[-2].offset_toward_target(corner_coords[sides['short'][second_corner]], dist_sweep)
                new_coord1 = traverse_coords[-1].offset_toward_target(corner_coords[sides['short'][closest_corner]], dist_sweep)
            self.log.debug('New traverse coordinates: {a}, {b} on iter {i}'.format(a=new_coord1,b=new_coord2, i=i))
            traverse_coords.extend([new_coord1,new_coord2])
    
        # use the coordinates to make waypoints at all altitudes
        waypoints = []
        for i, alt in enumerate(altitudes):
            if (i%2) == 0:
                for coord in traverse_coords:
                    waypoints.append(coord_to_waypoint(coord, alt))
            else:
                for coord in reversed(traverse_coords):
                    waypoints.append(coord_to_waypoint(coord, alt))

        self.log.debug('All waypoints calculated!')
    

        #begin flight
        self.start_log()
        try: 
            self.log.debug('Taking off to starting altitude {}m'.format(altitudes[0]))
            self.dc.take_off(altitudes[0])
            self.log.debug('Take off complete')

            for waypoint in waypoints:
                self.log.debug('Navigating to waypoint: ({lat}, {lon})'.format(
                    lat=waypoint.lat,
                    lon=waypoint.lon,
                ))
                self.dc.goto(coords=(waypoint.lat, waypoint.lon), altitude=waypoint.alt, airspeed=2)
                self.log.debug('Navigation to waypoint complete')

                location = self.dc.read_gps()
                self.log.debug('Arrived! Current location: ({lat}, {lon})'.format(
                    lat=location.lat,
                    lon=location.lon,
                ))

            self.log.info('Navigation to all waypoints complete. Landing now.')
            self.dc.land()
            self.log.info('Landed!')
        except FlightAbortedException:
            self.log.error('Flight aborted due to panic; aborting remaining tasks.')


    @panic
    def panic(self, *args, **kwargs):
        self.log.info('Mission panicked! Landing immediately.')
        self.dc.panic()


corners_names = {
    'eng': (
        (29.720318, -95.399884),
        (29.720397, -95.399709),
        (29.720534, -95.399780),
        (29.720467, -95.399917)),

    'soccer': (
        (29.717366, -95.405458),
        (29.717885, -95.405744),
        (29.717700, -95.406182),
        (29.717182, -95.405894)
    ),

    'rugby': (
        (29.718509, -95.406502),
        (29.718831, -95.405705),
        (29.718118, -95.406276),
        (29.718452, -95.405433)

    ),

    'academic': (
        (29.718334, -95.399128),
        (29.718655, -95.398189),
        (29.719051, -95.398361),
        (29.718730, -95.399230)
    ),

    'stadium': (
        (29.715915, -95.409058),
        (29.716742, -95.409077),
        (29.716733, -95.409571),
        (29.715906, -95.409563)
    ),
    'founders': (
        (29.719332, -95.397456),
        (29.719517, -95.396971),
        (29.719072, -95.397326),
        (29.719260, -95.396843),
    ),
    'wiess-1': (
        (29.715738, -95.402825),
        (29.715954, -95.402316),
        (29.715360, -95.402635),
        (29.715577, -95.402126),
    ),
    'wiess-2': (
        (29.714975, -95.402510),
        (29.715196, -95.402003),
        (29.714603, -95.402306),
        (29.714825, -95.401798),
    ),
}

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--fc-addr',
        dest='fc_addr',
        help='Address of the flight controller mavproxy messages',
        default=None,
    )

    parser.add_argument(
        '--log-file',
        dest='log_file',
        help='Path to the log file to create',
        default='{year}_{month}_{day}_{hour}_{min}_{sec}-travel-path.log'.format(
        	year=time.localtime()[0],
        	month=time.localtime()[1],
        	day=time.localtime()[2],
        	hour=time.localtime()[3],
        	min=time.localtime()[4],
        	sec=time.localtime()[5],
        	),  
    )
    args = parser.parse_args()

    TraversalMission(
        fc_addr=args.fc_addr,
        log_file=args.log_file,
    )

