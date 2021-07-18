import argparse
import time
import util

from skycommand.command import send
from skycommand.discovery import discover_drones
from skyengine.drone import DroneController
from skyengine.exceptions import FlightAbortedException
from skylog.logger import DirectLogger
from skylog.message import BaseMessage
from skymission.concurrency import tick
from skymission.mission import Mission
from skymission.mission import callback
from skymission.mission import panic

from geo import Coordinate
from geo import Waypoint
from formation import FormationOffset
from constants import VERTICAL_SEPARATION
from constants import STAGGER_LAUNCH_SEC
from constants import MIN_LATERAL_SEPARATION
from constants import MAX_DISTANCE_FROM_REGION
from constants import REGIONS

from simulation import IS_SIMULATION
from simulation import IS_SDR_ACTIVE


if IS_SIMULATION:
    import random
else:
    from rtlsdr import RtlSdr
    import numpy as np

class LocationMessage(BaseMessage):
    """
    Message for the drone's current location, speed, heading direction, and sdr readings.
    """

    def __init__(self, timestamp, location, speed, heading, sdr, dBm):
        self.timestamp = timestamp
        self.lat = location.lat
        self.lon = location.lon
        self.alt = location.alt
        self.speed = speed
        self.heading = heading
        self.sdr = sdr
        self.dBm = dBm

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
            'IQ': self.sdr,
            'dBm':self.dBm,
        }


class FormationTraversalMission(Mission):

    port = 4002
    mission_id = 'formation-traversal'
    drone_id = util.uuid()

    def __init__(self, fc_addr, log_file):
        """
        Create a FormationTraversalMission and start the mission server.

        :param fc_addr: MAVProxy address of the flight controller.
        :param log_file: Name of the log file for location data.
        """
        self.enable_disk_event_logging()

        self.dc = DroneController(fc_addr)
        self.logger = DirectLogger(path=log_file)
        self.log.debug('Drone controller and logger initialized successfully')

        # Show initial drones visible for debugging
        # Drone data of the form (ip, port)
        self.all_drones = discover_drones(local=True)
        self.other_drones = None
        self.this_drone = None
        self.log.debug(
            'Discovered {num_drones} drones in system (includes self): {neighbors}'.format(
                num_drones=len(self.all_drones),
                neighbors=self.all_drones,
            )
        )

        self.formation_offset = None # the offset of the drone from the formation center

         # IP addresses of the drones that have sent completion messages for the current task
        self.completion_messages = set()

        if IS_SIMULATION == False and IS_SDR_ACTIVE == True :

            self.sdr = RtlSdr()
            self.log.debug('Configuring RTL-SDR sensor.')
            self.sdr.sample_rate = 2.048e6  # Hz
            self.sdr.center_freq = 563e6     # Hz
            self.freq_correction = 60   # PPM
            self.sdr.gain = 10

        if IS_SIMULATION == False and IS_SDR_ACTIVE == False :
            self.log.debug('Running mission without RTL-SDR sensor.')

        if IS_SIMULATION == True :
            self.log.warn('Running mission as simulation. SDR will not be used')


        self.SamplesToDiscard = 10
        self.ContSamples = 0

        self.log.info('Formation-Waypoints mission initialization complete')
        self.start_server()

    @tick()
    def start_log(self):
        """
        Start periodically logging the drone GPS location, speed, and direction.
        """
        gps_pos = self.dc.read_gps()
        nSamples = 256 # DONT CHANGE TO 128!!!!! IT CAUSES KERNEL PANIC (unless you change tick or find another fix)
        self.ContSamples +=1
        if IS_SIMULATION:
            dBm = random.uniform(-2, -3)
            time.sleep(0.1)
        else:
            samples = self.sdr.read_samples(nSamples)
            dBm = 10 * np.log10(np.mean(np.power(np.abs(samples), 2)))

        if self.ContSamples > self.SamplesToDiscard:
            self.logger.log(LocationMessage(
                    timestamp=time.time(),
                    location=self.dc.read_gps(),
                    speed=self.dc.vehicle.airspeed,
                    heading=self.dc.vehicle.heading,
                    sdr=[""],
                    dBm=dBm
            ))

    @callback(
        endpoint='/start-mission',
        description='Discovers location of all drones and tell drones to launch.',
        required_params=('region_name', 'min_alt', 'hover_duration', 'speed'),
        public=True,
    )
    def start_mission(self, data, *args, **kwargs):
        """
        Client invoked endpoint to begin the formation waypoints mission.
        This endpoint should only be invoked on one drone, which could be viewed as the 'master' drone.
        :param data: Required to be of the form:
            {
                'region_name' : ... # Name of the region to fly around
                'min_alt': 0.00,  # Minimum altitude (some of the drones will fly at higher altitudes)
                'hover_duration' : 0.00,
                'speed': 0.00
            }
        """
        if data['region_name'] not in REGIONS:
            self.log.warn(
                'Region ID {region_id} is unknown; expected to be one of {regions}'.format(
                    region_id=data['region_name'],
                    regions=REGIONS.keys(),
                )
            )
            self.log.warn('Aborting.')
            return

        self.discover_drones()
        self.share_discovery()
        drone_locations = self.find_drone_locations()
        center_location = self.find_center(drone_locations)

        if self.are_too_close(drone_locations):
            self.log.warn('Drones are too close! Locations: {locations}'.format(
                locations=drone_locations
            ))
            self.log.warn('Aborting.')
            return

        if self.is_too_far_from_region(center_location, data['region_name']):
            self.log.warn('Drones are too far from region. Drone center: {center} Region center {region}'.format(
                center=center_location,
                region=REGIONS[data['region_name']].center()
            ))
            self.log.warn('Aborting.')
            return

        self.broadcast_center_loc(center_location)
        self.launch_drones(data, center_location)

        self.log.debug('Mision start complete')

    @callback(
        endpoint='/launch',
        description='Visits the waypoints plus the offset from center.',
        required_params=('coordinates', 'alt', 'hover_duration', 'speed'),
        public=False,
    )
    def launch(self, data, *args, **kwargs):
        """
        Neighbor invoked endpoint to launch the mission (intended to be invoked by master drone)
        :param data: Required to be of the form:
            {
                'coordinates': [...],     # Coordinates to visit as (lat, lon) tuples
                'alt': 0.00 # Altitude that this drone will fly at
                'hover_duration' : 0.00,
                'speed': 0.00
            }
        """
        coordinates = [Coordinate.deserialize(coord) for coord in data['coordinates']]
        self.log.info('Coordinates to visit: {coordinates}'.format(coordinates=coordinates))

        hover_duration = data['hover_duration']
        altitude = data['alt']
        speed = data['speed']

        self.log.debug('Beginning launch with alt: {alt}, hover_duration: {hover_duration}, speed: {speed}'.format(
                alt=altitude,
                hover_duration=hover_duration,
                speed=speed
            ))


        self.log.debug('Taking off to altitude: {alt}'.format(alt=altitude))
        self.dc.take_off(altitude)
        self.log.debug('Take off complete')
        self.broadcast_completion()
        self.wait_for_others()
        self.log.debug('Finished waiting. Starting log')
        
        if IS_SDR_ACTIVE:
            self.start_log()

        for coord in coordinates:
            self.visit_with_offset(coord, altitude, speed)
            self.broadcast_completion()
            self.wait_for_others()
            self.hover_for(hover_duration)

        # TODO: return to original location to land

        self.log.info('Navigation to all waypoints complete. Landing now.')
        self.dc.land()
        self.log.info('Landed!')

    def discover_drones(self):
        """
        Discovers all participating drones.
        This is intended to be called by the master drone, who can then share the
        discovery with all the other drones. This avoids each drone individually performing
        their own discoveries, which may end up being inconsistent.

        This modifies self.all_drones, self.other_drones and self.this_drone


        :returns: none
        """
        # Find all_drones again even though they are found in __init__
        # This is because new drones may have been initialized since this drone ran __init__ 
        self.all_drones = set(discover_drones(local=True))
        self.other_drones = set(discover_drones(local=False))
        self.this_drone = self.all_drones.difference(self.other_drones)

        self.log.debug(
            'Discovered {num_drones} drone(s) in system (includes self): {neighbors}'.format(
                num_drones=len(self.all_drones),
                neighbors=list(self.all_drones),
            )
        )

        # Make sure that each drone is discovered only once
        # Drones may be discovered multiple times during simulation if multiple IP addresses are detected.
        # There's also a small chance this might happen due to a network error.
        normalized = {}
        for drone in self.all_drones:
            drone_ip, drone_port = drone
            remote_id = send(
                drone_ip=drone_ip,
                mission_id=self.mission_id,
                endpoint='/identity',
                skyserve_port=drone_port,
            ).json().get('data', {}).get('id')
            normalized[remote_id] = drone
            if remote_id == self.drone_id:
                self.this_drone = drone

        self.all_drones = normalized.values()

        self.log.info(
            'Verified and normalized all neighbors. Drone IDs and addresses: {all_drones}'.format(
                all_drones=self.all_drones,
            )
        )

    def share_discovery(self):
        """
        Tells all participating drones about all of the discovered drones
        This is intended to be called by the master drone after it has 
        finished discovering the other drones

        :returns: none
        """
        for drone in self.other_drones:
            drone_ip, drone_port = drone
            data = {
                'all_drones': list(self.all_drones),
                'target_ip': drone_ip,
                'target_port': drone_port
            }

            send(
                drone_ip=drone_ip,
                mission_id=self.mission_id,
                endpoint='/receive-discovery-data',
                data=data,
                skyserve_port=drone_port,
            )

    @callback(
        endpoint='/identity',
        description='Query the local drone\'s ID and type.',
        public=False,
    )
    def identity(self, *args, **kwargs):
        """
        Reads the local drone ID.
        """
        return {
            'id': self.drone_id,
        }

    @callback(
        endpoint='/find-offset',
        description='Calculates offset from center',
        required_params=('lat', 'lon'),
        public=False,
    )
    def find_offset_from_center(self, data, *args, **kwargs):
        """
        Calculates the bearing and distance from the center of the drone cluster.
        The bearing points FROM the center TOWARDS the drone.
        This modifies self.offset_distance and self.offset_bearing
        :param data: Required to be of the form:
            {
                'lat': 0.00,  # Center latitude
                'lon': 0.00,  # Center longitude
            }
        """
        self.log.debug('Received center location: {data}. Calculating offset'.format(data=data))

        center_coord = Coordinate(data['lat'], data['lon'])
        local_coord = Coordinate.from_gps_data(self.dc.read_gps())

        self.formation_offset = FormationOffset(
                bearing = center_coord.bearing_toward(local_coord),
                distance = center_coord.distance_to(local_coord),
            )

        self.log.info(
            'Found offset from center: {offset}'.format(
                offset = self.formation_offset
            )
        )

    def find_drone_locations(self):
        """
        Finds the coordinates of all drones

        :returns: dictionary of drone coordinates, in the form of 
                {
                    '0.0.0.0' : coordinate1,
                    '0.0.0.1' : coordinate2 
                }
        """
        drone_coords = {}
        for drone in self.all_drones:
            drone_ip, drone_port = drone
            location_data = send(
                drone_ip=drone_ip,
                mission_id=self.mission_id,
                endpoint='/coordinate-msg',
                skyserve_port=drone_port,
            ).json().get('data', {})
            drone_coord = Coordinate(location_data['lat'], location_data['lon'])
            drone_coords[drone_ip] = drone_coord

        self.log.info(
            'Found drone coordinates: {coords}'.format(
                coords=drone_coords,
            )
        )

        return drone_coords

    @callback(
        endpoint='/coordinate-msg',
        description='Query the local drone\'s location.',
        public=False,
    )
    def coordinate_msg(self, *args, **kwargs):
        """
        Reads the local drone location coordinates.
        """
        self.log.debug('Sending current coordinate.')
        local_location = self.dc.read_gps()

        return {
            'lat': local_location.lat,
            'lon': local_location.lon
        }

    def find_center(self, drone_coords):
        """
        Finds the center of the drone cluster.
        This is intended to be run by the master drone
        :param drone_coords: dictionary of drone coordinates, in the form of 
                {
                    '0.0.0.0' : coordinate1,
                    '0.0.0.1' : coordinate2 
                }
        :returns a Coordinate of the center location
        """
        lat_sum = 0
        lon_sum = 0

        for coord in drone_coords.values():
            lat_sum += coord.lat
            lon_sum += coord.lon

        center_lat = lat_sum / len(drone_coords)
        center_lon = lon_sum / len(drone_coords)

        center_coord = Coordinate(center_lat, center_lon)

        self.log.info(
            'Found cluster center: {coord}'.format(
                coord=center_coord,
            )
        )

        return center_coord

    def broadcast_center_loc(self, center_location):
        """
        Tells each drone the location of the drone cluster's center.
        This is intended to be run by the master drone
        """
        self.log.debug('Broadcasting center location')

        location_data = {'lat': center_location.lat, 'lon': center_location.lon}
        for drone in self.all_drones:
            drone_ip, drone_port = drone
            self.log.debug('Broadcasting center to {ip}'.format(ip=drone_ip))
            send(
                drone_ip=drone_ip,
                mission_id=self.mission_id,
                endpoint='/find-offset',
                data=location_data,
                skyserve_port=drone_port,
            )

    def launch_drones(self, data, center_location):
        """
        Launches each of the drones on the mission at different altitudes.
        This is intended to be run by the master drone
        :param data: same as data for /start-mission endpoint
        :param center_location: A Coordinate of the current center of the drone formation
        """
        # Start at the max altitude to avoid collisions during ascent
        altitude = data['min_alt'] + VERTICAL_SEPARATION * len(self.all_drones)
        coordinates = self.make_serialized_coordinates(data['region_name'], center_location)

        for drone_ip, drone_port in self.all_drones:
            altitude -= VERTICAL_SEPARATION # Separate each drone altitude by VERTICAL_SEPARATION
            launch_data = {
                'coordinates': coordinates,
                'alt': altitude,
                'hover_duration': data['hover_duration'],
                'speed': data['speed']
            }
            self.log.debug('Sending launch command to: {ip}'.format(ip=drone_ip))
            send(
                drone_ip=drone_ip,
                mission_id=self.mission_id,
                endpoint='/launch',
                data=launch_data,
                skyserve_port=drone_port,
                async=True,
            )
            time.sleep(STAGGER_LAUNCH_SEC)

    def broadcast_completion(self):
        """
        Broadcasts to all of the drones that this drone has completed the current task
        """
        msg_data={'ip_addr': self.this_drone[0], 'id': self.drone_id}
        for drone in self.other_drones:
            drone_ip, drone_port = drone
            self.log.debug('Broadcasting completion to {ip}:{port}'.format(
                ip=drone_ip, 
                port=drone_port
            ))
            send(
                drone_ip=drone_ip,
                mission_id=self.mission_id,
                endpoint='/receive-completion',
                skyserve_port=drone_port,
                data=msg_data,
                async=True,
            )

    def visit_with_offset(self, coordinate, altitude, speed):
        """
        Fly to a coordinate moved by the given offset from center
        :param coordinate: the coordinate to fly to
        :param altitude: the altitude to fly to
        :param speed: the speed to fly the mission at
        """
        coordinate = self.formation_offset.offset_coordinate(coordinate)

        self.dc.goto(coords=(coordinate.lat, coordinate.lon), altitude=altitude, airspeed=speed)
            
        self.log.debug('Navigation to waypoint complete')
        location = self.dc.read_gps()
        self.log.debug('Arrived! Current location: ({lat}, {lon})'.format(
            lat=location.lat,
            lon=location.lon,
        ))

    def wait_for_others(self):
        """
        Sleep until receiving 'task-complete' messages from all drones
        """
        self.log.debug('Waiting for other drones')
        
        while len(self.completion_messages) < len(self.other_drones):
            time.sleep(0.01)
            # TODO: add maximum wait time. Land if it takes too long (maybe 30 seconds)  

        self.log.debug('Done waiting. Received completion messages from: {senders}'.format(
            senders=self.completion_messages
        ))
        # Reset list of received messages
        self.completion_messages = set()

    def hover_for(self, duration):
        """
        Hover for a particular length of time
        :param duration: the number of seconds to hover
        """
        self.log.debug('Hovering for {duration} seconds'.format(
            duration=duration,
        ))
        time.sleep(duration)

    def are_too_close(self, drone_locations):
        """
        Determines whether the drones are placed too close to each other
        :param drone_locations: dictionary of drone coordinates, in the form of 
                {
                    '0.0.0.0' : coordinate1,
                    '0.0.0.1' : coordinate2 
                }
        :returns True if they are too close, False if they are a safe distance apart
        """
        too_close = False
        for ip, coord in drone_locations.items():
            for ip2, coord2 in drone_locations.items():
                if ip == ip2:
                    continue

                if coord.distance_to(coord2) < MIN_LATERAL_SEPARATION:
                    too_close = True
        return too_close

    def is_too_far_from_region(self, center_location, region_name):
        """
        Determines whether the drones are placed too far from the designated region to fly
        :param center_location: a Coordinate instance describing the center of the drone cluster
        :param region_name: the name of the region that the drones are scheduled to fly in
        :returns True if they are too far, False if they are reasonably close
        """
        region = REGIONS[region_name] # Region name already verified
        return center_location.distance_to(region.center()) > MAX_DISTANCE_FROM_REGION

    @callback(
        endpoint='/receive-discovery-data',
        description='Receives data about all drones discovered by the master drone.',
        required_params=('all_drones', 'target_ip', 'target_port'),
        public=False,
    )

    def receive_discovery_data(self, data, *args, **kwargs):
        """
        Neighbor invoked endpoint to receive data about all of the drones discovered by the master
        :param data: Required to be of the form:
            {
                'all_drones': [(ip1, port1), (ip2, port2)], # All participating drones
                'target_ip':..., # the ip address of this drone
                'target_port':..., # the SkyServe port of this drone
            }
        """
        self.log.debug('Received discovery data: {data}'.format(data=data))
        
        # Turn 2D list into set of tuples
        self.all_drones=set([
            (drone_data[0], drone_data[1]) 
            for drone_data in data['all_drones']])

        self.this_drone = (data['target_ip'], data['target_port'])
        self.other_drones = set(self.all_drones)
        self.other_drones.remove(self.this_drone)

    @callback(
        endpoint='/receive-completion',
        description='Receives a notification that the sender has completed its current task.',
        required_params=('ip_addr', 'id'), # Added 2nd param id b/c having only 1 required param breaks the callback
        public=False,
    )
    def receive_completion_notification(self, data, *args, **kwargs):
        """
        Neighbor invoked endpoint to launch the mission
        :param data: none
        """
        self.log.debug('Received completion message from {ip}'.format(
           ip = data['ip_addr']
        ))
        self.completion_messages.add(data['ip_addr'])

    def make_serialized_coordinates(self, region_name, center_location):
        """
        Makes the coordinates that the cluster of drones should fly to during the mission.
        They will fly around the border of the given region.

        :param region_name: the name of the region that the drones will be flying in
        :param center_location: a Coordinate of the center location of the drone formation
        :returns: a list of json serialized coordinates that represent the coordinates that the formation
        should fly to during the mission
        """
        region = REGIONS[region_name] # Region name already verified
        region = region.shrink(10) # shrink

        coordinates = region.survey_coordinates(center_location)
        coordinates = [coord.serialize() for coord in coordinates]

        return coordinates

    @panic
    def panic(self, *args, **kwargs):
        self.log.info('Mission panicked! Landing immediately.')
        self.dc.panic()


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
        default='path-log_formation-traversal_{year}_{month}_{day}_{hour}_{min}_{sec}'.format(
                year=time.localtime()[0],
                month=time.localtime()[1],
                day=time.localtime()[2],
                hour=time.localtime()[3],
                min=time.localtime()[4],
                sec=time.localtime()[5],
                )
,
    )
    args = parser.parse_args()

    FormationTraversalMission(
        fc_addr=args.fc_addr,
        log_file=args.log_file,
    )
