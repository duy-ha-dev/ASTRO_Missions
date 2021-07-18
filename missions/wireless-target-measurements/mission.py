import multiprocessing
import time
import argparse
import numpy as np
import util
import json

from math import sqrt

from skycommand.command import send
from skycommand.discovery import discover_drones
from skyengine.drone import DroneController
from skyengine.exceptions import AvionicsException
from skyengine.exceptions import FlightAbortedException
from skyengine.exceptions import RicePublicRelationsException
from skyengine.gpsdata import GPSData
from skylog.logger import DirectLogger
from skymission.concurrency import tick
from skymission.mission import Mission
from skymission.mission import callback
from skymission.mission import panic
from dronekit import mavutil

from constants import REGIONS
from constants import GROUNDTRUTH_ALPHA
from constants import GROUNDTRUTH_EPSILON
from geo import Coordinate
from sensing_message import SensingMessage
from simulation import IS_SIMULATION


if IS_SIMULATION:
    import random
    from simulation import TARGET_LOCATION
else:
    from rtlsdr import RtlSdr

MAX_TOLERABLE_REGION_SEPARATION_DISTANCE = 150


class GDParameters(object):
    """
    Parameters used as input to the gradient descent algorithm.
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


def coord_to_waypoint(coordinate, alt):
    """
    Takes an instance of Coordinate and its altitude and outputs an instance of Waypoint
    """
    return Waypoint(coordinate.lat, coordinate.lon, alt)


class WirelessTargetMission(Mission):
    """
    Survey a defined quadrilateral region, looking for wireless target.
    """
    port = 4000
    mission_id = 'wireless-target-virtual-drone'
    drone_id = util.uuid()

    def __init__(self, fc_addr):
        """
        Mission initialization procedure.

        :param fc_addr: Flight controller address.
        """
        self.enable_disk_event_logging()
        self.log.debug('Initializing mission...')

        # Flag indicating whether the drone should abort any in-progress surveys
        # Not necessary if https://code.skynet.engineering/T204 is fixed
        self.should_abort_survey = multiprocessing.Value('i', False)

        self.sensed_data = []
        self.airspeed = 0
        self.region = None

        self.dc = DroneController(fc_addr)
        self.logger_sensed_raw = DirectLogger(path='{year}_{month}_{day}_{hour}_{min}_{sec}-sensed_raw.log'.format(
                                                year=time.localtime()[0],
                                                month=time.localtime()[1],
                                                day=time.localtime()[2],
                                                hour=time.localtime()[3],
                                                min=time.localtime()[4],
                                                sec=time.localtime()[5],
                                          ))

        if IS_SIMULATION:
            self.log.warn('Running mission as simulation. SDR will not be used')
        else:
            self.sdr = RtlSdr()
            self.log.debug('Configuring RTL-SDR sensor.')
            self.sdr.sample_rate = 2.048e6  # Hz
            self.sdr.center_freq = 563e6     # Hz
            self.freq_correction = 60   # PPM
            self.sdr.gain = 10.0  #Supported gain values (29): 0.0 0.9 1.4 2.7 3.7 7.7 8.7 12.5 14.4 15.7 16.6 19.7 20.7 22.9 25.4 28.0 29.7 32.8 33.8 36.4 37.2 38.6 40.2 42.1 43.4 43.9 44.5 48.0 49.6

        self.sdr_samples_to_discard = 10
        self.sdr_samples_number = 0
        self.sensor_range = 40 #in meters
        self.alpha = None
        self.epsilon = None
        self.dBm = None

        self.log.info('Mission initialization complete!')

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

    @callback(
        endpoint='/start-mission',
        description='Start the mission: take off the drone and start sensing',
        required_params=('region_id', 'alt', 'airspeed', 'hover_time'),
        public=True,
    )
    def start(self, data, *args, **kwargs):
        """
        This is the main mission entry point.
        """
        self.airspeed = data['airspeed']
        alt = data['alt'][0]
        # self.swarm_altitude = alt
        self.log.debug('Taking off: altitude {alt}.'.format(alt=alt))
        self.dc.take_off(alt)
        self.dc.move_by((0.5,1), alt) #needed yaw cannot be set in guided mode until the first movement command has been received.
        self.wireless_sensing()

        starting_time = time.time()
        hover_time = data['hover_time']
        while (time.time() - starting_time) < hover_time:
            current_heading = self.dc.vehicle.heading
            self.condition_yaw(90, relative=True)
            time.sleep(25)

        self.log.info('Measures complete; begin landing')
        self.dc.land()
        self.log.info('Landed!')

    @tick()
    def wireless_sensing(self):
        """
        Periodically perform a scan with the wireless sensor, updating internal mission state with
        the sensed data values.
        """
        gps_pos = self.dc.read_gps()
        heading = self.dc.vehicle.heading
        n_samples = 256  # DON'T CHANGE TO 128!!!!! IT CAUSES KERNEL PANIC (unless you change tick or find another fix)
        if IS_SIMULATION:
            dBm = self.get_simulated_dBm()
            self.dBm = dBm
            self.sdr_samples_number += 1
            time.sleep(0.1)
        else:
            self.sdr_samples_number += 1
            samples = self.sdr.read_samples(n_samples)
            dBm = 10 * np.log10(np.mean(np.power(np.abs(samples), 2)))
            self.dBm = dBm

        if self.sdr_samples_number > self.sdr_samples_to_discard:
            data_to_log = SensingMessage(
                location=gps_pos,
                dBm=dBm,
                heading=heading
            )
            self.logger_sensed_raw.log(data_to_log)
            self.sensed_data.append(data_to_log)

    @callback(
        endpoint='/land',
        description='Land the drone.',
        public=True,
    )
    def land(self, *args, **kwargs):
        """
        Land the drone after mission completion. This call is GCS-initiated.
        """
        self.log.info('Landing drone...')
        self.dc.land()
        self.log.info('Landed!')

    @panic
    def panic(self, *args, **kwargs):
        """
        This mission's panic behavior delegates to Skyengine.
        """
        self.log.error('Panicking drone!')
        self.dc.panic()

    def region_survey_waypoints(self, corner_coords, altitudes):
        """
        Generate waypoints used to fully survey a specified region.

        :param corner_coords: A list of Coordinate instances.
        :param altitudes: A list of altitudes for the drone to sweep at
        :return: A list of Waypoint objects representing the locations to visit, in order, to perform
                 a full survey of the region.
        """

        dist = [corner_coords[-1].distance_to(corner_coords[i]) for i in range(3)]

        # use the distances in order to pair the corner indeces with the neighbors
        dist_copy = dist[:]
        dist_copy.sort()
        neighbor_close = dist.index(min(dist))
        neighbor_far = dist.index(dist_copy[1])
        neighbor_diag = dist.index(dist_copy[2])
        sides = dict()
        sides['short'] = {3: neighbor_close,
                          neighbor_close: 3,
                          neighbor_far: neighbor_diag,
                          neighbor_diag: neighbor_far}
        sides['long'] = {3: neighbor_far,
                         neighbor_far: 3,
                         neighbor_close: neighbor_diag,
                         neighbor_diag: neighbor_close}

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

        # make the coordinates for the traversal points of rotation
        traverse_coords = [closest_corner_coord, second_corner_coord]
        shortest_side_dist = min([closest_corner_coord.distance_to(corner_coords[sides['short'][closest_corner]]),
                                  second_corner_coord.distance_to(corner_coords[sides['short'][second_corner]])])
        num_of_sweeps = int(round(shortest_side_dist / self.sensor_range))
        if num_of_sweeps != 0:
            dist_sweep = shortest_side_dist / float(num_of_sweeps)
        else:
            dist_sweep = 0
        self.log.debug('Number of sweeps: {a} Distance per sweep: {b}'.format(a=num_of_sweeps, b=dist_sweep))

        for i in range(num_of_sweeps):
            if (i % 2) == 0:
                new_coord1 = traverse_coords[-1].offset_toward_target(corner_coords[sides['short'][second_corner]],
                                                                      dist_sweep)
                new_coord2 = traverse_coords[-2].offset_toward_target(corner_coords[sides['short'][closest_corner]],
                                                                      dist_sweep)
            else:
                new_coord2 = traverse_coords[-2].offset_toward_target(corner_coords[sides['short'][second_corner]],
                                                                      dist_sweep)
                new_coord1 = traverse_coords[-1].offset_toward_target(corner_coords[sides['short'][closest_corner]],
                                                                      dist_sweep)
            # self.log.debug('New traverse coordinates: {a}, {b} on iter {i}'.format(a=new_coord1, b=new_coord2, i=i))
            traverse_coords.extend([new_coord1, new_coord2])

        # use the coordinates to make waypoints at all 3 altitudes
        waypoints = []
        for i, alt in enumerate(altitudes):
            if (i % 2) == 0:
                for coord in traverse_coords:
                    waypoints.append(coord_to_waypoint(coord, alt))
            else:
                for coord in reversed(traverse_coords):
                    waypoints.append(coord_to_waypoint(coord, alt))

        self.log.debug('All waypoints calculated!')

        return waypoints

    def get_simulated_dBm(self):
        """
        Return a simulated dBm value that takes into account a moving target
        """
        target_height = 1.0 # meters

        current_local = self.dc.read_gps()
        current_pos = Coordinate.from_gps_data(current_local)

        horizontal_distance = current_pos.distance_to(Coordinate(TARGET_LOCATION[0],TARGET_LOCATION[1]))
        vertical_distance = current_local.alt - target_height
        distance_to_target = sqrt(vertical_distance**2 + horizontal_distance**2)

        dBm = GROUNDTRUTH_ALPHA * np.log10(distance_to_target) + GROUNDTRUTH_EPSILON

        dBm += np.random.normal(0,2) # Add random noise to the data

        return dBm


    def condition_yaw(self, heading, relative=False):
        """
        Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).
        This method sets an absolute heading by default, but you can set the `relative` parameter
        to `True` to set yaw relative to the current yaw heading.
        By default the yaw of the vehicle will follow the direction of travel. After setting
        the yaw using this function there is no way to return to the default yaw "follow direction
        of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)
        For more information see:
        http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
        """
        if relative:
            is_relative = 1 #yaw relative to direction of travel
        else:
            is_relative = 0 #yaw is an absolute angle
        # create the CONDITION_YAW command using command_long_encode()
        msg = self.dc.vehicle.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
            0, #confirmation
            heading,    # param 1, yaw in degrees
            0,          # param 2, yaw speed deg/s
            1,          # param 3, direction -1 ccw, 1 cw
            is_relative, # param 4, relative offset 1, absolute angle 0
            0, 0, 0)    # param 5 ~ 7 not used
        # send command to vehicle
        self.dc.vehicle.send_mavlink(msg)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--fc-addr',
        dest='fc_addr',
        help='Address of the flight controller mavproxy messages',
    )
    args = parser.parse_args()

    mission = WirelessTargetMission(fc_addr=args.fc_addr)
    mission.start_server()
