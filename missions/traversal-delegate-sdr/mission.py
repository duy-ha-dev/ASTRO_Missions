import argparse
import time
import numpy as np
from math import sqrt
import util
import json

from skycommand.command import send
from skycommand.discovery import discover_drones
from skyengine.drone import DroneController
from skyengine.exceptions import FlightAbortedException
from hotspot_message import HotspotWirelessMessage
from hotspot_message import HotspotFilterMessage
from hotspot_message import UsedData
from skylog.logger import DirectLogger
from skylog.message import BaseMessage
from skymission.concurrency import tick
from skymission.mission import Mission
from skymission.mission import callback
from skymission.mission import panic
from simulation import IS_SIMULATION
from constants import SIMULATED_ALPHA
from constants import SIMULATED_EPSILON

from kalman import KalmanFilter
import gradient_descent
from geo import Coordinate
from geo import Region
from constants import REGIONS

if IS_SIMULATION:
    import random
    from simulation import TARGET_LOCATION
else:
    from rtlsdr import RtlSdr

# Time, in seconds, that we wait between delegating regions to drones.
# This helps guarantee that we don't send them on a collision course with one
# another.
_STAGGER_LAUNCH_SEC = 5.0

NUM_MIN_WIRELESS_SAMPLES = 250
if IS_SIMULATION:
    NUM_MIN_WIRELESS_SAMPLES = 100

SAMPLES_SWARM = 200
GD_THRESHOLD_ERROR = 50
THRESHOLD_dBm = -40.0
THRESHOLD_Sampling = 0.03
NUM_LOCATION_COMPUTED = 1

class GDParameters(object):
    """
    Parameters used as input to the hotspot localization algorithm.
    """
    NUM_ITERATIONS = 50
    NUM_EPOCH = 50
    THRESHOLD = 0.1
    LEARNING_RATE = 0.001

class Waypoint:
    def __init__(self, lat, lon, alt):
        self.lat = lat
        self.lon = lon
        self.alt = alt


class TraversalMessage(BaseMessage):
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

    mission_id = 'delegate-traversal-sdr'
    port = 4000
    drone_id = util.uuid()

    def __init__(self, fc_addr, log_file):
        """
        Create a TraversalMission and start the mission server.

        :param fc_addr: MAVProxy address of the flight controller.
        :param log_file: Name of the log file for location data.
        """
        self.enable_disk_event_logging()

        self.dc = DroneController(fc_addr)
        self.logger = DirectLogger(path=log_file)
        self.traversal_sensing = DirectLogger(path='{year}_{month}_{day}_{hour}_{min}_{sec}-traversal_sensing-log'.format(
                                                year=time.localtime()[0],
                                                month=time.localtime()[1],
                                                day=time.localtime()[2],
                                                hour=time.localtime()[3],
                                                min=time.localtime()[4],
                                                sec=time.localtime()[5],
                                                ))

        self.traversal_sensing_raw = DirectLogger(path='{year}_{month}_{day}_{hour}_{min}_{sec}-traversal_sensing_raw.log'.format(
                                                year=time.localtime()[0],
                                                month=time.localtime()[1],
                                                day=time.localtime()[2],
                                                hour=time.localtime()[3],
                                                min=time.localtime()[4],
                                                sec=time.localtime()[5],
                                          ))

        self.used_data_kf_filter_logger = DirectLogger(path='{year}_{month}_{day}_{hour}_{min}_{sec}-gd_data_kf.log'.format(
                                                year=time.localtime()[0],
                                                month=time.localtime()[1],
                                                day=time.localtime()[2],
                                                hour=time.localtime()[3],
                                                min=time.localtime()[4],
                                                sec=time.localtime()[5],
                                            ))

        self.used_data_logger = DirectLogger(path='{year}_{month}_{day}_{hour}_{min}_{sec}-gd_data.log'.format(
                                                year=time.localtime()[0],
                                                month=time.localtime()[1],
                                                day=time.localtime()[2],
                                                hour=time.localtime()[3],
                                                min=time.localtime()[4],
                                                sec=time.localtime()[5],
                                            ))
        self.log.debug('Drone controller and logger initialized successfully')

        if IS_SIMULATION:
            self.log.warn('Running mission as simulation. SDR will not be used')
        else:
            self.sdr = RtlSdr()

            self.log.debug('Configuring RTL-SDR sensor.')
            self.sdr.sample_rate = 2.048e6  # Hz
            self.sdr.center_freq = 563e6     # Hz
            self.freq_correction = 60   # PPM
            self.sdr.gain = 10.0  #Supported gain values (29): 0.0 0.9 1.4 2.7 3.7 7.7 8.7 12.5 14.4 15.7 16.6 19.7 20.7 22.9 25.4 28.0 29.7 32.8 33.8 36.4 37.2 38.6 40.2 42.1 43.4 43.9 44.5 48.0 49.6

        self.SamplesToDiscard = 10
        self.ContSamples = 0
        self.lastInsert = 0
        self.FLAG = -1
        self.airspeed = 0


        self.wireless_data = []

        self.neighbors = discover_drones(local=True)
        self.log.debug(
            'Discovered {num_drones} drones in system (includes self): {neighbors}'.format(
                num_drones=len(self.neighbors),
                neighbors=self.neighbors,
            )
        )

        self.start_server()

    @tick()
    def start_wireless_sensing(self):
        """
        Periodically perform a scan with the wireless sensor, updating internal mission state with
        the sensed data values.
        """
        gps_pos = self.dc.read_gps()
        n_samples = 256  # DON'T CHANGE TO 128!!!!! IT CAUSES KERNEL PANIC (unless you change tick or find another fix)
        if IS_SIMULATION:
            dBm = self.get_simulated_dBm()
            self.dBm = dBm
            self.ContSamples += 1
            time.sleep(0.1)
        else:
            self.ContSamples += 1
            samples = self.sdr.read_samples(n_samples)
            dBm = 10 * np.log10(np.mean(np.power(np.abs(samples), 2)))
            self.dBm = dBm

        if self.ContSamples > self.SamplesToDiscard:

            wireless_msg0 = HotspotWirelessMessage(
                location=gps_pos,
                sdr=[],
                dBm=dBm,
                heading=self.dc.vehicle.heading,
            )
            self.traversal_sensing_raw.log(wireless_msg0)

            now = time.time()

            if ((dBm > THRESHOLD_dBm) and (now - self.lastInsert) >= THRESHOLD_Sampling):
                if self.FLAG == 2:
                    wireless_msg = HotspotWirelessMessage(
                        location=self.dc.read_gps(),
                        sdr=[],
                        dBm=self.dBm,
                )
                    self.traversal_sensing.log(wireless_msg)

                if self.FLAG == 1:
                    wireless_msg2 = HotspotFilterMessage(
                        hotcaltime=self.hottime,
                        alepcaltime=self.aleptime,
                        survetime=self.surveytime,
                        swarmtime=self.swarmtime,
                        FLAG=self.FLAG,
                        location=self.dc.read_gps(),
                        sdr=[],
                        dBm=self.dBm,
                    )

                    self.traversal_sensing.log(wireless_msg2)

                self.wireless_data.append(wireless_msg0)

                self.lastInsert = time.time()


    @callback(
        endpoint='/start-mission',
        description='Start the delegated traversal mission: delegating out the region to all discoverable drones',
        required_params=('corners', 'alt', 'airspeed'),
        public=True,
    )
    def start_mission(self, data, *args, **kwargs):
        """
        Client invoked endpoint to begin the delegated traversal mission

        :param data: Required to be of the form:
                     {
                         'corners': ..., #Name of set of corners matching desired region
                         'alt': [...] #List of altitudes in meters
                         'airspeed': #int
                     }
        """
        region_id = data['corners']
        self.airspeed = data['airspeed']
        if region_id not in REGIONS:
            self.log.warn(
                'Region ID {region_id} is unknown; expected to be one of {regions}'.format(
                    region_id=region_id,
                    regions=REGIONS.keys(),
                )
            )
            self.log.warn('Aborting.')
            return

        # Deduplicate neighbors with the same drone ID
        self.normalize_neighbors()

        # Make sure that the drone doesn't think it's really far from the requested survey region
        local_location = Coordinate.from_gps_data(self.dc.read_gps())
        dist_to_region_center = REGIONS[region_id].center().distance_to(local_location)
        if dist_to_region_center > 150:
            self.log.warn('Too far from survey region center: {dist_to_region_center} m!'.format(
                dist_to_region_center=dist_to_region_center,
            ))
            self.log.warn('Aborting.')
            return

        # Discover all participating drones.
        other_drones = set(discover_drones(local=False))
        all_drones = set(self.neighbors)
        this_drone = all_drones.difference(other_drones)
        self.log.debug("Discovered {a} drones: {b}".format(a=len(all_drones), b=all_drones))

        # Try again if we managed to detect more than one drone as the current one.
        # There's a small chance this might happen due to a network error.
        if (len(this_drone) != 1):
            self.log.error("Error discovering drones, trying again.")

            self.start_mission(data, *args, **kwargs)
            return

        # Make sure the current drone is last to leave.
        drones = list(other_drones) + list(this_drone)

        partition_regions = REGIONS[region_id].vertical_partition(len(drones))
        dist = [local_location.distance_to(partition.center()) for partition in partition_regions]
        final_region_order = [x for _,x in sorted(zip(dist, partition_regions), reverse=True)]

        self.log.debug('Itineraries complete! Sending them out now...')

        for i, (drone_ip, drone_port) in enumerate(drones):
            send(
                drone_ip=drone_ip,
                mission_id=self.mission_id,
                endpoint='/delegation-region',
                data={'corner-coords': final_region_order[i].corners(),
                        'alt': data['alt'],
                        'airspeed': self.airspeed,
                        },
                skyserve_port=drone_port,
                async=True
                )
            # Delay between launches.
            time.sleep(_STAGGER_LAUNCH_SEC)


    @callback(
        endpoint='/delegation-region',
        description='One drone accepts its delegated region, sweeping it starting from'
                    'nearest corner while reporting location, speed, and heading.',
        required_params=('corner-coords', 'alt', 'airspeed'),
        public=False,
    )
    def delegated_region_sweep(self, data, *args, **kwargs):
        """
        Sweep through the delegated region and collect information in log

        :param data: Required to be of the form:
                     {
                         'corner-coords': ..., #List coordinates which represent the corners of region
                         'alt': [...] #List of altitudes in meters
                         'airspeed': #int
                     }
        """
        self.log.debug('Delegated mission begin')
        altitudes = data['alt']
        self.airspeed = data['airspeed']
        corner_coords = [Coordinate(c[0],c[1]) for c in data['corner-coords']]

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
                corner_coords[a] = corner_coords[a].offset_toward_target(corner_coords[b], 5)

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
            if (i % 2) == 0:
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
        try:
            self.log.debug('Taking off to starting altitude {}m'.format(altitudes[0]))
            self.dc.take_off(altitudes[0])
            self.log.debug('Take off complete and starting log')
            self.start_wireless_sensing()
            self.localize_target()

            for waypoint in waypoints:
                self.log.debug('Navigating to waypoint: ({lat}, {lon})'.format(
                    lat=waypoint.lat,
                    lon=waypoint.lon,
                ))
                self.dc.goto(coords=(waypoint.lat, waypoint.lon), altitude=waypoint.alt, airspeed=self.airspeed)
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

    @tick(interval=1)
    def localize_target(self):
        """
        Attempt to localize the hotspot location based on the collected data.

        :return: The hotspot location, expressed as a GPSData object, if available. None otherwise,
                 e.g. in the event that there does not exist enough samples yet to make an informed
                 decision.
        """
        len_data = len(self.wireless_data)

        if len_data < NUM_MIN_WIRELESS_SAMPLES:
            self.log.warn('Not enough data points to estimate hotspot location, only {} samples.'.format(len_data))
            return

        self.log.debug('Calculating hotspot with data points {a} to {b}'.format(
            a=len_data-NUM_MIN_WIRELESS_SAMPLES,
            b=len_data
        ))
        # Reshape the locally cached data into the expected matrix format
        samples = np.zeros(shape=(NUM_MIN_WIRELESS_SAMPLES, 4))
        samples_kf_filtered = np.zeros(shape=(NUM_MIN_WIRELESS_SAMPLES, 4))

        kf = KalmanFilter(0.008, 10)

        for idx, sample in enumerate(self.wireless_data[(len_data - NUM_MIN_WIRELESS_SAMPLES):len_data]):
            samples[idx][0] = sample.lat
            samples[idx][1] = sample.lon
            samples[idx][2] = sample.alt
            samples[idx][3] = sample.dBm
            samples_kf_filtered[idx][0] = sample.lat
            samples_kf_filtered[idx][1] = sample.lon
            samples_kf_filtered[idx][2] = sample.alt
            samples_kf_filtered[idx][3] = kf.filter(sample.dBm)

        used_data_kf_filter = json.dumps([{
            'lat': i[0],
            'lon': i[1],
            'alt': i[2],
            'dBm': i[3],
        	} for i in samples_kf_filtered])

        used_data = json.dumps([{
            'lat': i[0],
            'lon': i[1],
            'alt': i[2],
            'dBm': i[3],
        	} for i in samples])

        log_used_data = UsedData (
        	used_data = used_data
        )
        log_used_data_kf_filter = UsedData (
        	used_data = used_data_kf_filter
        )

        self.used_data_logger.log(log_used_data)
        self.used_data_kf_filter_logger.log(log_used_data_kf_filter)

        # Calculate the hotspot location
        start = time.time()
        hotspot_location, _ = gradient_descent.RSSI_Loc_BatchedGD_5par(
            data=samples_kf_filtered,
            numEpoch=GDParameters.NUM_EPOCH,
            numIterations=GDParameters.NUM_ITERATIONS,
            threshold=GDParameters.THRESHOLD,
            learning_rate=GDParameters.LEARNING_RATE,
        )
        end = time.time()
        self.hottime = end - start

        self.log.info('Calculated hotspot : lat= {lat}, lon = {lon}, alt = {alt}'.format(
                        lat=hotspot_location[0],
                        lon=hotspot_location[1],
                        alt=hotspot_location[2],
                    )
        )


    @callback(
        endpoint='/identity',
        description='Query the local drone\'s ID and type.',
        public=False,
    )
    def identity(self, *args, **kwargs):
        """
        Read the local drone ID.
        """
        return {
            'id': self.drone_id,
            }

    def get_simulated_dBm(self):
        """
        Return a simulated dBm value that takes into account a moving cheater
        """

        cheater_height = 1.0 # meters

        current_local = self.dc.read_gps()
        current_pos = Coordinate.from_gps_data(current_local)

        horizontal_distance = current_pos.distance_to(Coordinate(TARGET_LOCATION[0],TARGET_LOCATION[1]))
        vertical_distance = current_local.alt - cheater_height
        distance_to_hotspot = sqrt(vertical_distance**2 + horizontal_distance**2)

        dBm = SIMULATED_ALPHA * np.log10(distance_to_hotspot) + SIMULATED_EPSILON

        dBm += np.random.normal(0,2) # Add random noise to the data

        return dBm

    def normalize_neighbors(self):
        """
        Normalize the IP addresses of all detected neighbors. This mutates the internal neighbors
        property.
        """
        self.log.debug('Verifying health of all neighbors.')

        normalized = {}
        for neighbor in self.neighbors:
            drone_ip, drone_port = neighbor
            remote_id = send(
                drone_ip=drone_ip,
                mission_id=self.mission_id,
                endpoint='/identity',
                skyserve_port=drone_port,
            ).json().get('data', {}).get('id')
            normalized[remote_id] = neighbor

        self.neighbors = normalized.values()

        self.log.debug(
            'Verified and normalized all neighbors. Drone IDs and addresses: {normalized}'.format(
                normalized=normalized,
            )
        )


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
        default='{year}_{month}_{day}_{hour}_{min}_{sec}-traversal-path.log'.format(
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
