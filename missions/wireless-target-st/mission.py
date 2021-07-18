import multiprocessing
import time
import argparse
from math import sqrt

import json

import numpy as np
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
from skymission.concurrency import async_process

import gradient_descent
import util
from swarm import predict
from constants import REGIONS
from constants import SHOULD_WAIT
from constants import SIMULATED_ALPHA
from constants import SIMULATED_EPSILON
from constants import HARDCODE_ALPHA
from constants import IS_ALPHA_CONTINUOUS
from geo import Coordinate
from formation import FormationOffset
from hotspot_message import HotspotWirelessMessage
from hotspot_message import HotspotFilterMessage
from hotspot_message import ExchangedData

from simulation import IS_SIMULATION


from scipy.signal import savgol_filter
from kalman import KalmanFilter

if IS_SIMULATION:
    import random
else:
    from rtlsdr import RtlSdr

# Number of meters vertically separating each drone in the system performing a survey.
# Set this to zero for all drones to survey at the same altitude.
# Raise this number to increase the vertical distance between each drone, for purposes of safety.
DRONE_ALTITUDE_MULTIPLIER = 2

# Base altitude for drones to swarm at
SUMMON_BASE_ALT = 5

SAMPLES_SL = 500
SAMPLES_SWARM = 150

GD_THRESHOLD_ERROR = 50
THRESHOLD_dBm = -40.0
THRESHOLD_Sampling = 0.0
NUM_LOCATION_COMPUTED = 1
SUMMON_OFFSET = 5

# The maximum tolerable number of meters away which the drone can be from the center of the
# requested survey region. This exists as a safety check in case the drone believes it is much
# farther away from the survey region than it actually is.
MAX_TOLERABLE_REGION_SEPARATION_DISTANCE = 150
if IS_SIMULATION:
    MAX_TOLERABLE_REGION_SEPARATION_DISTANCE = 500

# Delay before the simulated cheater moves. This allows the drones to take off before it starts moving.
MOVE_CHEATER_DELAY = 30.0

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


def coord_to_waypoint(coordinate, alt):
    """
    Takes an instance of Coordinate and its altitude and outputs an instance of Waypoint
    """
    return Waypoint(coordinate.lat, coordinate.lon, alt)


class HotspotIdentificationMission(Mission):
    """
    Survey a defined quadrilateral region, looking for wireless hotspots. Once a drone is able to
    consistently identify a hotspot, it will summon all other drones in the system to its location.
    Then the drones will begin to share data and relocalize the spectrum cheater, moving in a formation.
    """
    port = 4000
    mission_id = 'wireless-target-st'
    drone_id = util.uuid()

    def __init__(self, fc_addr):
        """
        Mission initialization procedure.

        :param fc_addr: Flight controller address.
        """
        self.enable_disk_event_logging()

        self.log.info('Local drone ID is {drone_id}.'.format(drone_id=self.drone_id))
        self.log.debug('Initializing mission...')

        self.neighbors = set(discover_drones(local=True))
        self.other_drones = set(discover_drones(local=False))
        self.this_drone = list(self.neighbors.difference(self.other_drones))[0]
        self.log.debug(
            'Discovered {num_drones} drones in system: {others} and self: {me}'.format(
                num_drones=len(self.neighbors),
                others=self.other_drones,
                me=self.this_drone,
            )
        )

        # IP addresses of the drones that have sent completion messages for the current task
        self.completion_messages = set()
        # Summoner is the drone that summons the others to the hotspot and is then in
        # charge of calculating and sharing new hotspot locations during swarm and track
        self.is_master = False
        self.master_drone = tuple()

        # Flag indicating whether the drone should abort any in-progress surveys
        # Not necessary if https://code.skynet.engineering/T204 is fixed
        self.should_abort_survey = multiprocessing.Value('i', False)
        self.should_avoid_summoning = multiprocessing.Value('i', False)

        self.wireless_data = []
        self.hotspots = []
        self.airspeed = 0
        self.region = None

        self.ready_to_send = False

        self.st_type = None

        self.FLAG = -1
        # -1: Not reading samples
        #  1: Swarm and Track
        #  2: Search and Learn
        # TODO: Change to enum with meaningful names

        self.dc = DroneController(fc_addr)
        self.flight_logger = DirectLogger(path='{year}_{month}_{day}_{hour}_{min}_{sec}-wireless_target.log'.format(
                                                year=time.localtime()[0],
                                                month=time.localtime()[1],
                                                day=time.localtime()[2],
                                                hour=time.localtime()[3],
                                                min=time.localtime()[4],
                                                sec=time.localtime()[5],
                                          ))
        self.wireless_filter = DirectLogger(path='{year}_{month}_{day}_{hour}_{min}_{sec}-swarm_track.log'.format(
                                                year=time.localtime()[0],
                                                month=time.localtime()[1],
                                                day=time.localtime()[2],
                                                hour=time.localtime()[3],
                                                min=time.localtime()[4],
                                                sec=time.localtime()[5],
                                            ))
        self.shared_data_kf_filter_logger = DirectLogger(path='{year}_{month}_{day}_{hour}_{min}_{sec}-shared_data_kf.log'.format(
                                                year=time.localtime()[0],
                                                month=time.localtime()[1],
                                                day=time.localtime()[2],
                                                hour=time.localtime()[3],
                                                min=time.localtime()[4],
                                                sec=time.localtime()[5],
                                            ))
        self.shared_data_logger = DirectLogger(path='{year}_{month}_{day}_{hour}_{min}_{sec}-shared_data.log'.format(
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
            self.sdr.gain = 10 #Supported gain values (29): 0.0 0.9 1.4 2.7 3.7 7.7 8.7 12.5 14.4 15.7 16.6 19.7 20.7 22.9 25.4 28.0 29.7 32.8 33.8 36.4 37.2 38.6 40.2 42.1 43.4 43.9 44.5 48.0 49.6


        self.SamplesToDiscard = 10 # TODO: make higher?
        self.ContSamples = 0

        self.lastInsert = 0
        self.sentData = 1

        self.alpha = None
        self.survey_start_time = None
        self.epsilon = None
        self.swarm_altitude = None  # Height that this drone will fly at when summoned/swarming
        self.dBm = None
        self.aleptime = None
        self.hottime = None
        self.surveytime = None
        self.swarmtime = None

        self.formation_offset = None

        self.all_drones_data = dict()
        self.cheater_start_time = 0

        self.times_received = 0

        self.initial_simulated_hotspot = None
        self.current_simulated_hotspot = None
        self.hotspot_meters_moved = 0

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
        endpoint='/neighbors',
        description='Query the neighbor drones detected by the drone during initialization.',
        public=True,
    )
    def neighbors(self, *args, **kwargs):
        """
        Read the neighbors currently known to the local drone.
        """
        return {
            'neighbors': [
                {'ip': ip, 'port': port}
                for ip, port in self.neighbors
            ],
        }

    @callback(
        endpoint='/start-mission',
        description='Start the mission: assign a survey region for all drones, take off the '
                    'local drone, and request take off of all neighbor drones',
        required_params=('region_id', 'alt', 'airspeed'),
        public=True,
    )
    def start(self, data, *args, **kwargs):
        """
        The input parameters are:

            - region_id (str): One of the region IDs defined in the constant REGIONS
            - alt (list): The survey altitudes. Every active drone will try to fly at these
                             altitudes
            - airspeed (int): Airspeed for all drones at which the survey should be performed.
            - type: random, formation
            - hover_time
        """
        self.is_master = True
        self.airspeed = data['airspeed']
        self.st_type = data['type']
        region_id = data['region_id']

        drone_locations = self.find_drone_locations()
        center_location = self.find_center(drone_locations)

        self.log.info('Requested mission start ({type}) for drone ID {drone_id}.'.format(
            type=self.st_type,
            drone_id=self.drone_id,
        ))

        if self.st_type == "hover":
            start_time = time.time() + MOVE_CHEATER_DELAY
            for idx, drone in enumerate(self.neighbors):
                drone_ip, drone_port = drone

                msg = 'Requesting hover for drone {drone_ip}:{drone_port} at ' \
                      'airspeed {airspeed}'
                send(
                    drone_ip=drone_ip,
                    mission_id=self.mission_id,
                    endpoint='/hover',
                    data={
                        'region_id': region_id,
                        'alt': [c + idx * DRONE_ALTITUDE_MULTIPLIER for c in data['alt']],
                        'center_lat': center_location.lat,
                        'center_lon': center_location.lon,
                        'hover_time': data['hover_time'],
                        'master_drone_ip': self.this_drone[0],
                        'master_drone_port': self.this_drone[1],
                        'airspeed': self.airspeed,
                        'start_time': start_time,
                    },
                    skyserve_port=drone_port,
                    async=True,
                )
                time.sleep(3)
        elif self.st_type == "spin":
            start_time = time.time() + MOVE_CHEATER_DELAY
            for idx, drone in enumerate(self.neighbors):
                drone_ip, drone_port = drone

                msg = 'Requesting hover for drone {drone_ip}:{drone_port} at ' \
                      'airspeed {airspeed}'
                send(
                    drone_ip=drone_ip,
                    mission_id=self.mission_id,
                    endpoint='/spin',
                    data={
                        'region_id': region_id,
                        'alt': [c + idx * DRONE_ALTITUDE_MULTIPLIER for c in data['alt']],
                        'center_lat': center_location.lat,
                        'center_lon': center_location.lon,
                        'hover_time': data['hover_time'],
                        'master_drone_ip': self.this_drone[0],
                        'master_drone_port': self.this_drone[1],
                        'airspeed': self.airspeed,
                        'start_time': start_time,
                    },
                    skyserve_port=drone_port,
                    async=True,
                )
                time.sleep(3)
        elif self.st_type == "track":
            start_time = time.time() + MOVE_CHEATER_DELAY
            for idx, drone in enumerate(self.neighbors):
                drone_ip, drone_port = drone

                msg = 'Requesting hover for drone {drone_ip}:{drone_port} at ' \
                      'airspeed {airspeed}'
                send(
                    drone_ip=drone_ip,
                    mission_id=self.mission_id,
                    endpoint='/track',
                    data={
                        'region_id': region_id,
                        'alt': [c + idx * DRONE_ALTITUDE_MULTIPLIER for c in data['alt']],
                        'center_lat': center_location.lat,
                        'center_lon': center_location.lon,
                        'hover_time': data['hover_time'],
                        'master_drone_ip': self.this_drone[0],
                        'master_drone_port': self.this_drone[1],
                        'airspeed': self.airspeed,
                        'start_time': start_time,
                    },
                    skyserve_port=drone_port,
                    async=True,
                )
                time.sleep(3)
        elif self.st_type == "simple-track":
            start_time = time.time() + MOVE_CHEATER_DELAY
            for idx, drone in enumerate(self.neighbors):
                drone_ip, drone_port = drone

                msg = 'Requesting hover for drone {drone_ip}:{drone_port} at ' \
                      'airspeed {airspeed}'
                send(
                    drone_ip=drone_ip,
                    mission_id=self.mission_id,
                    endpoint='/simple-track',
                    data={
                        'region_id': region_id,
                        'alt': [c + idx * DRONE_ALTITUDE_MULTIPLIER for c in data['alt']],
                        'center_lat': center_location.lat,
                        'center_lon': center_location.lon,
                        'hover_time': data['hover_time'],
                        'master_drone_ip': self.this_drone[0],
                        'master_drone_port': self.this_drone[1],
                        'airspeed': self.airspeed,
                        'start_time': start_time,
                    },
                    skyserve_port=drone_port,
                    async=True,
                )
                time.sleep(3)

    @callback(
        endpoint='/hover',
        description='Start hovering',
        required_params=('region_id', 'alt', 'hover_time', 'master_drone_ip', 'master_drone_port'),
        public=False,
    )
    def run_hover_version(self, data, *args, **kwargs):
        self.master_drone = (data['master_drone_ip'], data['master_drone_port'])

        alt = data['alt']

        center_coord = Coordinate(data['center_lat'], data['center_lon'])
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

        self.initial_simulated_hotspot = center_coord


        # begin flight
        self.region=REGIONS[data['region_id']]
        current_local = self.dc.read_gps()
        altitudes = data['alt']
        self.log.debug('Taking off: altitude {alt}'.format(alt=altitudes[0]))
        self.dc.take_off(altitudes[0])
        self.start_wireless_sensing()
        self.FLAG = 1
        self.log.debug('Take off complete and beginning log')
        self.log.debug('Hovering for {} seconds'.format(data['hover_time']))
        if self.is_master:
            self.log.info('Local drone beginning shared wireless data')
            self.swarm_track()
        time.sleep(float(data['hover_time']))
        for altitude in altitudes[1:]:
            try:
                self.log.debug('Changing altitude to {}'.format(altitude))
                self.dc.goto(coords=(current_local.lat,current_local.lon), altitude=altitude, airspeed=2)
                self.log.debug('Hovering for {} seconds'.format(data['hover_time']))
                time.sleep(data['hover_time'])
            except FlightAbortedException:
                self.log.error('Flight aborted due to panic; aborting remaining tasks.')
        self.is_mission_complete = True
        self.log.debug('Drone landing')
        self.dc.land()

    @callback(
        endpoint='/spin',
        description='Take off and start moving in a circle around the given center',
        required_params=('region_id', 'alt', 'center_lat', 'center_lon', 'master_drone_ip', 'master_drone_port', 'airspeed'),
        public=False,
    )
    def run_spin_version(self, data, *args, **kwargs):
        self.master_drone = (data['master_drone_ip'], data['master_drone_port'])
        self.region=REGIONS[data['region_id']]

        center_coord = Coordinate(data['center_lat'], data['center_lon'])
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

        self.initial_simulated_hotspot = center_coord


        #begin flight
        altitudes = data['alt']
        speed = data['airspeed']
        self.log.debug('Taking off: altitude {alt}'.format(alt=altitudes[0]))
        self.dc.take_off(altitudes[0])
        self.wait_for_others()
        self.start_wireless_sensing()
        self.FLAG = 1
        self.log.debug('Take off complete and beginning log')
        if self.is_master:
            self.log.info('Local drone beginning shared wireless data')
            self.swarm_track()

        circle_slices = 8
        angle_size = 360.0 / float(circle_slices)
        revolution_count = 2
        for alt in altitudes:
            self.log.debug('Performing {rev} spins at altitude: {alt}'.format(
                rev=revolution_count,
                alt=alt))
            for i in range(revolution_count):
                for j in range(circle_slices):
                    self.formation_offset.rotate_by(angle_size)
                    self.visit_with_offset(center_coord, alt, speed)
                    self.wait_for_others()
                self.log.debug('Completed revolution number: {rev}'.format(rev=i))


        self.is_mission_complete = True
        self.log.debug('Drone landing')
        self.dc.land()

    @callback(
        endpoint='/simple-track',
        description='Swarm the hotspot while maintaining the current formation',
        required_params=('region_id', 'alt', 'center_lat', 'center_lon', 'hover_time', 'master_drone_ip', 'master_drone_port', 'airspeed', 'start_time'),
        public=False,
    )
    def run_simple_track_version(self, data, *args, **kwargs):
        self.st_type = "simple-track"

        self.master_drone = (data['master_drone_ip'], data['master_drone_port'])
        self.airspeed = data['airspeed']
        self.region = REGIONS[data['region_id']]
        self.cheater_start_time = data['start_time']

        self.alpha = SIMULATED_ALPHA
        self.epsilon = SIMULATED_EPSILON

        center_coord = Coordinate(data['center_lat'], data['center_lon'])
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

        self.initial_simulated_hotspot = center_coord

        alt = data['alt']
        self.swarm_altitude = alt[0]

        current_local = self.dc.read_gps()
        altitudes = data['alt']
        self.log.debug('Taking off: altitude {alt}'.format(alt=altitudes[0]))
        self.dc.take_off(altitudes[0])

        self.log.debug('Take off complete and beginning sensing')

        self.start_wireless_sensing()
        if SHOULD_WAIT:
            self.send_completion_notification()

        self.FLAG = 1

    @callback(
        endpoint='/track',
        description='Swarm the hotspot while maintaining the current formation',
        required_params=('region_id', 'alt', 'center_lat', 'center_lon', 'hover_time', 'master_drone_ip', 'master_drone_port', 'airspeed', 'start_time'),
        public=False,
    )
    def run_track_version(self, data, *args, **kwargs):
        self.master_drone = (data['master_drone_ip'], data['master_drone_port'])
        self.airspeed = data['airspeed']
        self.region = REGIONS[data['region_id']]
        self.cheater_start_time = data['start_time']

        # self.alpha = SIMULATED_ALPHA
        # self.epsilon = SIMULATED_EPSILON

        self.alpha = None
        self.epsilon = None

        center_coord = Coordinate(data['center_lat'], data['center_lon'])
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

        self.initial_simulated_hotspot = center_coord

        alt = data['alt']
        self.swarm_altitude = alt[0]

        current_local = self.dc.read_gps()
        altitudes = data['alt']
        self.log.debug('Taking off: altitude {alt}'.format(alt=altitudes[0]))
        self.dc.take_off(altitudes[0])

        self.log.debug('Take off complete and beginning sensing')

        self.start_wireless_sensing()

        if SHOULD_WAIT:
            self.send_completion_notification()

        if self.is_master:
            self.log.info('Local drone beginning shared wireless data')
            self.swarm_track()

        self.FLAG = 1

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
            # dBm = random.uniform(-1, -10)
            self.dBm = dBm
            self.ContSamples += 1
            time.sleep(0.01)
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
            )
            self.flight_logger.log(wireless_msg0)

            now = time.time()
            # if ((dBm > THRESHOLD_dBm) and (now - self.lastInsert) >= THRESHOLD_Sampling):

            if (now - self.lastInsert) >= THRESHOLD_Sampling:
                if self.FLAG == 2:
                    wireless_msg = HotspotWirelessMessage(
                        location=self.dc.read_gps(),
                        sdr=[],
                        dBm=self.dBm,
                )
                    self.wireless_logger.log(wireless_msg)

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

                    self.wireless_filter.log(wireless_msg2)

                self.wireless_data.append(wireless_msg0)

                self.lastInsert = time.time()
                # if len(self.wireless_data) >= SAMPLES_SWARM * self.sentData:
                if len(self.wireless_data) >= SAMPLES_SL:
                    # self.sentData +=1
                    self.ready_to_send = True
    @callback(
        endpoint='/share',
        description='Query the wireless samples collected by the local drone',
        public=False,
    )
    def share_data(self, *args, **kwargs):
        self.FLAG = 1
        kf = KalmanFilter(0.008, 10)
        if self.ready_to_send:
            if self.alpha == 0 or self.alpha is None or IS_ALPHA_CONTINUOUS:
                samples = np.zeros(shape=(SAMPLES_SL, 4))
                len_data = len(self.wireless_data)
                filtered_data = []

                for idx, sample in enumerate(self.wireless_data[len_data-SAMPLES_SL:len_data]):
                    samples[idx][0] = sample.lat
                    samples[idx][1] = sample.lon
                    samples[idx][2] = sample.alt
                    samples[idx][3] = sample.dBm

                if HARDCODE_ALPHA:
                    self.alpha = SIMULATED_ALPHA
                    self.epsilon = SIMULATED_EPSILON

                else:
                    self.log.debug('Calculating alpha and epsilon with last data points')
                    # Calculate the hotspot location
                    _, [self.alpha, self.epsilon] = gradient_descent.RSSI_Loc_BatchedGD_5par(
                        data=samples,
                        numEpoch=GDParameters.NUM_EPOCH,
                        numIterations=GDParameters.NUM_ITERATIONS,
                        threshold=GDParameters.THRESHOLD,
                        learning_rate=GDParameters.LEARNING_RATE,
                    )

            else:
                samples = np.zeros(shape=(SAMPLES_SWARM, 4))
                len_data = len(self.wireless_data)
                filtered_data = []

                for idx, sample in enumerate(self.wireless_data[len_data-SAMPLES_SWARM:len_data]):
                    samples[idx][0] = sample.lat
                    samples[idx][1] = sample.lon
                    samples[idx][2] = sample.alt
                    samples[idx][3] = sample.dBm

            self.log.debug('Alpha: {a} Epsilon: {e}'.format(a=self.alpha, e=self.epsilon))

            data_to_share_kf_filter = json.dumps([{
            'lat': i[0],
            'lon': i[1],
            'alt': i[2],
            'dBm': kf.filter(i[3]),
        	} for i in samples])

            data_to_share = json.dumps([{
            'lat': i[0],
            'lon': i[1],
            'alt': i[2],
            'dBm': i[3],
        	} for i in samples])
        else:
            self.alpha = 0
            self.epsilon = 0
            data_to_share = [0,0,0,0]
            data_to_share_kf_filter = [0,0,0,0]

        log_shared_data = ExchangedData (
        	alpha = self.alpha,
        	epsilon = self.epsilon,
        	shared_data = data_to_share
        )

        log_shared_data_kf_filter = ExchangedData (
        	alpha = self.alpha,
        	epsilon = self.epsilon,
        	shared_data = data_to_share_kf_filter
        )

        self.shared_data_logger.log(log_shared_data)
        self.shared_data_kf_filter_logger.log(log_shared_data_kf_filter)

        return self.alpha, self.epsilon, data_to_share_kf_filter

    @callback(
        endpoint='/receive-share-simple',
        description='Receive shared data for simple-track mission type',
        required_params=('dBm', 'lat', 'lon', 'alt', 'ip'),
        public=False,
    )
    def receive_share_simple(self, data, *args, **kwargs):
        self.log.debug('Received data from: {ip}'.format(ip=data['ip']))
        self.all_drones_data[data['ip']] = {
            'dBm': data['dBm'],
            'lat': data['lat'],
            'lon': data['lon'],
            'alt': data['alt'],
            'ip': data['ip']
        }
        self.times_received += 1

        if len(self.all_drones_data) >= len(self.neighbors):
            self.swarm_track()
            self.all_drones_data = dict() # erase data after used

    @callback(
        endpoint='/receive-share',
        description='Receive shared data',
        required_params=('alpha', 'epsilon', 'samples', 'ip'),
        public=False,
    )
    def receive_share(self, data, *args, **kwargs):
        self.log.debug('Received data from: {ip}'.format(ip=data['ip']))
        self.all_drones_data[data['ip']] = {
            'alpha': data['alpha'],
            'epsilon': data['epsilon'],
            'samples': data['samples']
        }
        self.times_received += 1

        if len(self.all_drones_data) >= len(self.neighbors):
            self.swarm_track()
            self.all_drones_data = dict() # erase data after used

    @tick(interval=1.0)
    def swarm_track(self, *args, **kwargs):
        """
        Combine the wireless data from another node in the system with that tracked locally.
        """
        if self.st_type == "track" and SHOULD_WAIT:
            if len(self.completion_messages) < len(self.neighbors):
                return

        # TODO: make all drones know the st_type. Now only master knows
        if self.st_type == "simple-track":
            positions = []
            # Using separate pos_dict because find_center accepts a dictionary as argument
            # Todo: make find_center applicable to lists
            pos_dict = dict()
            dBms = []

            for ip, data in self.all_drones_data.items():
                pos_dict[ip] = Coordinate(data['lat'], data['lon'])
                positions.append(pos_dict[ip])
                dBms.append(data['dBm'])

            max_dBm_index = dBms.index(max(dBms))
            center_coord = self.find_center(pos_dict)
            pos_with_max_dBm = positions[max_dBm_index]
            max_dBm_bearing = center_coord.bearing_toward(pos_with_max_dBm)

            move_distance = 3 # meters
            combo_hotspot = center_coord.offset_bearing(max_dBm_bearing, move_distance)
        else:
            alphas = []
            epsilons = []
            shared_data = []
            for drone_ip, drone_port in self.neighbors:
                self.log.debug('Requesting data from drone {}'.format(drone_ip))
                local_alpha, local_epsilon, samples = send(
                    drone_ip=drone_ip,
                    mission_id=self.mission_id,
                    endpoint='/share',
                    skyserve_port=drone_port,
                ).json().get('data', {})
                if local_alpha == 0.0 and local_epsilon == 0.0:
                    return
                self.log.debug('Received data')
                alphas.append(local_alpha)
                epsilons.append(local_epsilon)
                samples = [[sample['lat'],
                            sample['lon'],
                            sample['alt'],
                            sample['dBm']] for sample in json.loads(samples)]

                shared_data.append(samples)

            drone_count = len(self.neighbors)

            prediction = predict(dronenum=drone_count,
                                 maxRun=1,
                                 numIterations=GDParameters.NUM_ITERATIONS,
                                 numEpoch=GDParameters.NUM_EPOCH,
                                 threshold=GDParameters.THRESHOLD,
                                 learning_rate=GDParameters.LEARNING_RATE,
                                 numberBatch=1,
                                 data_length=SAMPLES_SWARM*drone_count)

            if 1 < drone_count <= 3:
                try:
                    start = time.time()
                    hotspot = prediction.swarm(drone_data=shared_data,
                                               alphas=alphas,
                                               epsilons=epsilons)
                    end = time.time()
                    self.swarmtime = end - start
                    self.log.debug('Drone is using data from {a} drones'.format(a=drone_count))
                except IndexError:
                    self.log.warn('Hotspot localization failed. Data not good enough.')
                    return
            else:
                self.log.warn('Drone Number Incorrect')
                return

            combo_hotspot = Coordinate(hotspot[0], hotspot[1])

        self.log.debug('=========================================================================')
        self.log.debug('Calculated new hotspot at location: {}'.format(combo_hotspot))
        # TODO: allow this to run in all mission types
        # This would require implementing simulated cheater in all types, not just track
        # if IS_SIMULATION and (self.st_type == "track" or self.st_type == "simple-track"):
        if IS_SIMULATION:
            error = combo_hotspot.distance_to(self.current_simulated_hotspot)
            self.log.debug('Simulated error: {err}, Simulated hotspot has moved {dist} meters to: {loc}'.format(
                    err=error,
                    dist=self.hotspot_meters_moved,
                    loc=self.current_simulated_hotspot
                ))
        self.log.debug('=========================================================================')

        if not self.region.contains(combo_hotspot) and not IS_SIMULATION:
            self.log.debug('New hotspot is out of region')
            return

        if self.st_type == "track" or self.st_type == "simple-track":
            self.completion_messages = set()

        if self.st_type != "hover" and self.st_type != "spin":
            for drone_idx, (drone_ip, drone_port) in enumerate(self.neighbors):
                    self.log.debug('Sending drone at IP {drone_ip} to new hotspot location.'.format(
                        drone_ip=drone_ip,
                    ))

                    send(
                        drone_ip=drone_ip,
                        mission_id=self.mission_id,
                        endpoint='/swarm',
                        data={
                            'lat': combo_hotspot.lat,
                            'lon': combo_hotspot.lon,
                        },
                        skyserve_port=drone_port,
                        async=True,
                    )

    @callback(
        endpoint='/swarm',
        description='Move drones to new hotspot location after data was combined',
        required_params=('lat', 'lon'),
        public=False,
    )
    def formation_swarm(self, data, *args, **kwargs):
        """
        Move drones as a unit to new hotspot location
        """
        hotspot_coord = Coordinate(data['lat'], data['lon'])
        self.log.debug('Local drone told to move to hotspot at: {coord}'.format(
            coord=hotspot_coord,
            ))

        try:
            self.visit_with_offset(
                coordinate=hotspot_coord,
                altitude=self.swarm_altitude,
                speed=self.airspeed
            )
            if SHOULD_WAIT:
                self.send_completion_notification()
            self.log.debug('Drone arrived')
        except AvionicsException:
            self.log.error('Drone reported inability to travel to target coordinate!')
            return
        except FlightAbortedException:
            self.log.error('Waypoint navigation aborted!')
            return
        except RicePublicRelationsException:
            msg = 'Target navigation coordinate {waypoint} is invalid! Error in waypoint ' \
                    'calculation or defined region is too large?'
            self.log.error(msg.format(waypoint=hotspot_coord.pair()))
            return

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
        for drone in self.neighbors:
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
            if remote_id == self.drone_id:
                self.this_drone = neighbor
                self.log.debug('Drone identified itself: {this_drone}'.format(this_drone=self.this_drone))

        self.neighbors = normalized.values()

        self.log.debug(
            'Verified and normalized all neighbors. Drone IDs and addresses: {normalized}'.format(
                normalized=normalized,
            )
        )

    def visit_with_offset(self, coordinate, altitude, speed):
        """
        Fly to a coordinate moved by the given offset from center
        :param coordinate: the coordinate to fly to
        :param altitude: the altitude to fly to
        :param speed: the speed to fly the mission at
        """
        coordinate = self.formation_offset.offset_coordinate(coordinate)

        if not self.region.contains(coordinate) and not IS_SIMULATION:
            self.log.warn('Coordinate not within region. Will not travel to it')
            return

        self.dc.goto(coords=(coordinate.lat, coordinate.lon), altitude=altitude, airspeed=speed)

        self.log.debug('Navigation to waypoint complete')
        location = self.dc.read_gps()
        self.log.debug('Arrived! Current location: ({lat}, {lon})'.format(
            lat=location.lat,
            lon=location.lon,
        ))

    def wait_for_others(self):
        """
        Tells the others that the current task is complete.
        Then sleeps until receiving 'task-complete' messages from all drones
        """
        self.log.debug('Waiting for other drones')

        # Tell other drones that this drone is complete
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

        # Don't return until this drone hears that all others are also complete
        while len(self.completion_messages) < len(self.other_drones):
            time.sleep(0.01)
            # TODO: add maximum wait time. Land if it takes too long (maybe 30 seconds)

        self.log.debug('Done waiting. Received completion messages from: {senders}'.format(
            senders=self.completion_messages
        ))
        # Reset list of received messages
        self.completion_messages = set()

    def send_completion_notification(self):
        """
        Tells the summoner drone that the drone has finished its current task
        """
        msg_data={'ip_addr': self.this_drone[0], 'id': self.drone_id}
        drone_ip, drone_port = self.master_drone
        self.log.debug('Sending completion ({msg}) to summoner at {ip}:{port}'.format(
            msg=msg_data,
            ip=drone_ip,
            port=drone_port
        ))

        send(
            drone_ip=drone_ip,
            mission_id=self.mission_id,
            endpoint='/receive-completion',
            data=msg_data,
            skyserve_port=self.this_drone[1]
            #async=True
        )

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

    def get_simulated_dBm(self):
        """
        Return a simulated dBm value that takes into account a moving cheater
        """
        cheater_speed = 0 # m/s
        cheater_bearing = 90.0 # degrees: 0=North, 90=East, 180=South, 270=West
        cheater_height = 1.0 # meters

        elapsed_time = time.time() - self.cheater_start_time
        # Don't move the cheater until the DELAY has passed
        if elapsed_time < 0:
            elapsed_time = 0

        self.hotspot_meters_moved = elapsed_time * cheater_speed

        current_local = self.dc.read_gps()
        current_pos = Coordinate.from_gps_data(current_local)

        self.current_simulated_hotspot = self.initial_simulated_hotspot.offset_bearing(cheater_bearing, self.hotspot_meters_moved)
        horizontal_distance = current_pos.distance_to(self.current_simulated_hotspot)
        vertical_distance = current_local.alt - cheater_height
        distance_to_hotspot = sqrt(vertical_distance**2 + horizontal_distance**2)

        dBm = SIMULATED_ALPHA * np.log10(distance_to_hotspot) + SIMULATED_EPSILON
        dBm += np.random.normal(0,2) # Add random noise to the data

        return dBm


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--fc-addr',
        dest='fc_addr',
        help='Address of the flight controller mavproxy messages',
    )
    args = parser.parse_args()

    mission = HotspotIdentificationMission(fc_addr=args.fc_addr)
    mission.start_server()
