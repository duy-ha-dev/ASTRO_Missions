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

from kalman import KalmanFilter
import gradient_descent
from swarm import predict
from constants import REGIONS
from constants import SHOULD_WAIT
from constants import SIMULATED_ALPHA
from constants import SIMULATED_EPSILON
from constants import HARDCODE_ALPHA
from constants import IS_ALPHA_CONTINUOUS
from constants import FORMATION_RADIUS
from geo import Coordinate
from formation import FormationOffset
from hotspot_message import HotspotWirelessMessage
from hotspot_message import HotspotFilterMessage
from hotspot_message import UsedData
from simulation import IS_SIMULATION
from formation import FormationOffset
from formation import DroneFormation

if IS_SIMULATION:
    import random
    from simulation import TARGET_LOCATION
else:
    from rtlsdr import RtlSdr

# Number of meters vertically separating each drone in the system performing a survey.
# Set this to zero for all drones to survey at the same altitude.
# Raise this number to increase the vertical distance between each drone, for purposes of safety.
DRONE_ALTITUDE_MULTIPLIER = 2

# Base altitude for drones to swarm at
SUMMON_BASE_ALT = 5

# Required total number of samples (across all nodes) before attempting to localize the hotspot.
NUM_MIN_WIRELESS_SAMPLES = 250
if IS_SIMULATION:
    NUM_MIN_WIRELESS_SAMPLES = 500

SAMPLES_SWARM = 200
GD_THRESHOLD_ERROR = 50
THRESHOLD_dBm = -40.0
THRESHOLD_Sampling = 0.03
NUM_LOCATION_COMPUTED = 1

SUMMON_OFFSET = 5

# The maximum tolerable number of meters away which the drone can be from the center of the
# requested survey region. This exists as a safety check in case the drone believes it is much
# farther away from the survey region than it actually is.
MAX_TOLERABLE_REGION_SEPARATION_DISTANCE = 150
if IS_SIMULATION:
    MAX_TOLERABLE_REGION_SEPARATION_DISTANCE = 500


class GDParameters(object):
    """
    Parameters used as input to the hotspot localization algorithm.
    """
    NUM_ITERATIONS = 10
    NUM_EPOCH = 500
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
    """
    port = 4000
    mission_id = 'wireless-target-sl'
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
        self.is_summoner = False
        self.summoner_drone = tuple()

        # Flag indicating whether the drone should abort any in-progress surveys
        # Not necessary if https://code.skynet.engineering/T204 is fixed
        self.should_abort_survey = multiprocessing.Value('i', False)
        self.should_avoid_summoning = multiprocessing.Value('i', False)

        self.wireless_data = []
        self.hotspots = []
        self.airspeed = 0
        self.region = None

        self.FLAG = -1

        self.dc = DroneController(fc_addr)
        self.flight_logger = DirectLogger(path='{year}_{month}_{day}_{hour}_{min}_{sec}-wireless-target-sl.log'.format(
                                                year=time.localtime()[0],
                                                month=time.localtime()[1],
                                                day=time.localtime()[2],
                                                hour=time.localtime()[3],
                                                min=time.localtime()[4],
                                                sec=time.localtime()[5],
                                          ))

        self.wireless_logger = DirectLogger(path='{year}_{month}_{day}_{hour}_{min}_{sec}-search-learn.log'.format(
                                                year=time.localtime()[0],
                                                month=time.localtime()[1],
                                                day=time.localtime()[2],
                                                hour=time.localtime()[3],
                                                min=time.localtime()[4],
                                                sec=time.localtime()[5],
                                            ))

        self.wireless_filter = DirectLogger(path='{year}_{month}_{day}_{hour}_{min}_{sec}-swarm-track.log'.format(
                                                year=time.localtime()[0],
                                                month=time.localtime()[1],
                                                day=time.localtime()[2],
                                                hour=time.localtime()[3],
                                                min=time.localtime()[4],
                                                sec=time.localtime()[5],
                                            ))

        self.used_data_kf_filter_logger = DirectLogger(path='{year}_{month}_{day}_{hour}_{min}_{sec}-used_data_kf.log'.format(
                                                year=time.localtime()[0],
                                                month=time.localtime()[1],
                                                day=time.localtime()[2],
                                                hour=time.localtime()[3],
                                                min=time.localtime()[4],
                                                sec=time.localtime()[5],
                                            ))
        self.used_data_logger = DirectLogger(path='{year}_{month}_{day}_{hour}_{min}_{sec}-used_data.log'.format(
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
            self.sdr.gain = 22.9  #Supported gain values (29): 0.0 0.9 1.4 2.7 3.7 7.7 8.7 12.5 14.4 15.7 16.6 19.7 20.7 22.9 25.4 28.0 29.7 32.8 33.8 36.4 37.2 38.6 40.2 42.1 43.4 43.9 44.5 48.0 49.6

        self.SamplesToDiscard = 10
        self.ContSamples = 0

        self.lastInsert = 0

        self.alpha = None
        self.survey_start_time = None
        self.epsilon = None
        self.swarm_altitude = None  # Height that this drone will fly at when summoned/swarming
        self.dBm = None
        self.aleptime = None
        self.hottime = None
        self.surveytime = None
        self.swarmtime = None

        self.should_use_formation = False
        self.swarm_formation_offset = None
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
        endpoint='/share',
        description='Query the wireless samples collected by the local drone',
        public=False,
    )
    def share_data(self, *args, **kwargs):
        self.FLAG = 1
        samples = np.zeros(shape=(SAMPLES_SWARM, 4))
        len_data = len(self.wireless_data)
        for idx, sample in enumerate(self.wireless_data[len_data-SAMPLES_SWARM:len_data]):
            samples[idx][0] = sample.lat
            samples[idx][1] = sample.lon
            samples[idx][2] = sample.alt
            samples[idx][3] = sample.dBm

        start = time.time()
        if self.alpha is None:
            self.log.debug('Calculating alpha and epsilon with last data points')
            # Calculate the hotspot location
            _, [self.alpha, self.epsilon] = gradient_descent.RSSI_Loc_BatchedGD_5par(
                data=samples,
                numEpoch=GDParameters.NUM_EPOCH,
                numIterations=GDParameters.NUM_ITERATIONS,
                threshold=GDParameters.THRESHOLD,
                learning_rate=GDParameters.LEARNING_RATE,
            )

        data_to_share = [
                message.serialize() for message in self.wireless_data[len_data-SAMPLES_SWARM:len_data]
            ]
        self.log.debug('Alpha: {a} Epsilon: {e}'.format(a=self.alpha, e=self.epsilon))
        end = time.time()
        self.aleptime = end - start
        return self.alpha, self.epsilon, data_to_share

    @callback(
        endpoint='/start-mission',
        description='Start the mission: assign a survey region for all drones, take off the '
                    'local drone, and request take off of all neighbor drones',
        required_params=('region_id', 'alt', 'airspeed', 'use_formation'),
        public=True,
    )
    def start(self, data, *args, **kwargs):
        """
        This is the main mission entry point. It should only be called on one drone in the system.
        This endpoint does a number of things:

            1. Normalize the detected neighbors, in the event that more than one drone has the same
               advertised IP. (This really only happens in SITL, since the interface bat0 doesn't
               exist locally.)
            2. Partition the search space according to the number of drones currently active in the
               system. Assign each drone (including itself, the local drone) a unique search region.
            3. Asynchronously request every drone in the system (including itself) to survey the
               calculated region partition.

        The input parameters are:

            - region_id (str): One of the region IDs defined in the constant REGIONS
            - alt (list): The survey altitudes. Every active drone will try to fly at these
                             altitudes
            - airspeed (int): Airspeed for all drones at which the survey should be performed.
            - use_formation (boolean): whether or not to use a predefined formation in swarm
        """
        self.airspeed = data['airspeed']
        self.log.info('Requested mission start for drone ID {drone_id}.'.format(
            drone_id=self.drone_id,
        ))

        # Make sure that the requested survey region is one of the predefined, blessed regions
        region_id = data['region_id']
        if region_id not in REGIONS:
            self.log.warn(
                'Region ID {region_id} is unknown; expected to be one of {regions}'.format(
                    region_id=region_id,
                    regions=REGIONS.keys(),
                )
            )
            self.log.warn('Aborting.')
            return

        # Make sure that the drone doesn't think it's really far from the requested survey region
        local_location = Coordinate.from_gps_data(self.dc.read_gps())
        dist_to_region_center = REGIONS[region_id].center().distance_to(local_location)
        if dist_to_region_center > MAX_TOLERABLE_REGION_SEPARATION_DISTANCE:
            self.log.warn('Drone is too far from region. Aborting mission.')
            return

        # All safety checks complete.
        # Deduplicate neighbors with the same drone ID
        self.normalize_neighbors()

        # Partition the search space among all drones in the system and request a survey.
        self.log.debug(
            '{num_drones} drones currently in the system. Partitioning region {region_id}.'.format(
                num_drones=len(self.neighbors),
                region_id=region_id,
            )
        )
        partition_regions = REGIONS[region_id].vertical_partition(len(self.neighbors))
        dist = [local_location.distance_to(partition.center()) for partition in partition_regions]
        final_region_order = [x for _, x in sorted(zip(dist, partition_regions), reverse=True)]

        formation = DroneFormation(
            drone_count=len(self.neighbors),
            radius=FORMATION_RADIUS)

        for idx, (partition, drone) in enumerate(zip(final_region_order, self.neighbors)):
            drone_ip, drone_port = drone

            msg = 'Requesting survey of {region} for drone {drone_ip}:{drone_port} at ' \
                  'airspeed {airspeed}'
            self.log.info(msg.format(
                region=partition,
                drone_ip=drone_ip,
                drone_port=drone_port,
                airspeed=data['airspeed'],
            ))

            send(
                drone_ip=drone_ip,
                mission_id=self.mission_id,
                endpoint='/survey',
                data={
                    'nw': {
                        'lat': partition.nw.lat,
                        'lon': partition.nw.lon,
                    },
                    'ne': {
                        'lat': partition.ne.lat,
                        'lon': partition.ne.lon,
                    },
                    'sw': {
                        'lat': partition.sw.lat,
                        'lon': partition.sw.lon,
                    },
                    'se': {
                        'lat': partition.se.lat,
                        'lon': partition.se.lon,
                    },
                    'region_id': region_id,
                    'alt': [c + idx * DRONE_ALTITUDE_MULTIPLIER for c in data['alt']],
                    'airspeed': data['airspeed'],
                    'swarm_formation_offset': formation.get_offset(idx).serialize(),
                    'use_formation': data['use_formation']
                },
                skyserve_port=drone_port,
                async=True,
            )
            time.sleep(5)

    @callback(
        endpoint='/survey',
        description='Assign a region/partition to this drone and start an airborne survey.',
        required_params=('nw', 'ne', 'sw', 'se', 'region_id', 'alt', 'airspeed', 'swarm_formation_offset', 'use_formation'),
        public=False,
    )
    def survey(self, data, *args, **kwargs):
        """
        Given a search region, take off and navigate through a series of (runtime-calculated)
        waypoints that fully cover the region. Acquire wireless strength readings in the background.
        Continuously, recalculate hotspot until location is consistent. Then abort navigation.
        """
        self.log.debug('Local drone assigned survey region! Calculating survey waypoints...')
        self.should_use_formation = data['use_formation']
        if self.should_use_formation:
            self.swarm_formation_offset = FormationOffset.deserialize(data['swarm_formation_offset'])
        start = time.time()
        alt = data['alt']
        self.airspeed = data['airspeed']
        self.region = REGIONS[data['region_id']]
        survey_region = [Coordinate(data['nw']['lat'], data['nw']['lon']),
                         Coordinate(data['ne']['lat'], data['ne']['lon']),
                         Coordinate(data['sw']['lat'], data['sw']['lon']),
                         Coordinate(data['se']['lat'], data['se']['lon'])]

        waypoints = self.region_survey_waypoints(
            corner_coords=survey_region,
            altitudes=alt,
        )
        self.swarm_altitude = alt[0]
        self.log.debug('Taking off: altitude {alt}. This will also be the swarm altitude'.format(alt=alt[0]))
        self.dc.take_off(alt[0])

        self.log.debug('Starting wireless sensing')
        self.start_wireless_sensing()
        self.wireless_samples_calculation()

        while not self.should_abort_survey.value:
            for waypoint in waypoints:
                if self.should_abort_survey.value:
                    self.log.debug('Aborting local survey.')
                    self.FLAG = 2
                    total = time.time()
                    self.surveytime = total - start
                    return
                try:
                    self.log.debug('Navigating to waypoint: {waypoint}'.format(waypoint=(waypoint.lat, waypoint.lon)))
                    self.dc.goto(
                        coords=(waypoint.lat, waypoint.lon),
                        altitude=waypoint.alt,
                        airspeed=data['airspeed'],
                    )
                except AvionicsException:
                    self.log.error('Drone reported inability to travel to target coordinate!')
                    return
                except FlightAbortedException:
                    self.log.error('Waypoint navigation aborted!')
                    return
                except RicePublicRelationsException:
                    msg = 'Target navigation coordinate {waypoint} is invalid! Error in waypoint ' \
                        'calculation or defined region is too large?'
                    self.log.error(msg.format(waypoint=waypoint))
                    return

            if not self.should_abort_survey.value:
                self.log.warn('Completed full survey without localizing a hotspot! Repeating region survey')

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
            )
            self.flight_logger.log(wireless_msg0)

            now = time.time()

            if ((dBm > THRESHOLD_dBm) and (now - self.lastInsert) >= THRESHOLD_Sampling):
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

    @tick(interval=1)
    def wireless_samples_calculation(self):
        """
        Periodically recalculate hotspot location
        """
        if not self.should_abort_survey.value:
            if len(self.wireless_data) > NUM_MIN_WIRELESS_SAMPLES:
                self.log.debug('Local drone has collected {} points'. format(len(self.wireless_data)))

                self.log.debug('Starting computation for hotspot localization...')
                new_hotspot = self.localize_target()
                if new_hotspot:
                    if not self.region.contains(Coordinate.from_gps_data(new_hotspot)) and not IS_SIMULATION:
                        self.log.debug('Outside of region - Calculated hotspot {}'.format(
                            Coordinate.from_gps_data(new_hotspot)
                        ))
                        return
                    else:
                        self.log.debug('Inside region - Calculated hotspot {}'.format(
                            Coordinate.from_gps_data(new_hotspot)
                        ))
                        self.hotspots.append(new_hotspot)

                # if there are multiple hotspots calculated
                if len(self.hotspots) > NUM_LOCATION_COMPUTED:
                    coord_hotspots = [Coordinate.from_gps_data(c) for c in self.hotspots]
                    dist = [Coordinate.from_gps_data(new_hotspot).distance_to(c) for c in coord_hotspots[:-1]]
                    self.log.info('Distance is {dist}'.format(dist=dist))
                    # if the minimum distance between different calculated hotspots is less than X m
                    if min(dist) <= GD_THRESHOLD_ERROR and not self.should_avoid_summoning.value:
                        self.should_abort_survey.value = True
                        self.log.debug('Final summon hotspot location is {}'.format(new_hotspot))
                        if IS_SIMULATION:
                            computed_location = Coordinate.from_gps_data(new_hotspot)
                            self.log.debug('=========================================================================')
                            error = computed_location.distance_to(Coordinate(TARGET_LOCATION[0],TARGET_LOCATION[1]))
                            self.log.debug('Simulated error: {err}'.format(
                                        err=error))
                            self.log.debug('=========================================================================')

                        for drone_idx, (drone_ip, drone_port) in enumerate(self.neighbors):
                            self.log.debug('Summoning drone with IP {drone_ip} to hotspot location.'.format(
                                drone_ip=drone_ip
                            ))
                            send(
                                drone_ip=drone_ip,
                                mission_id=self.mission_id,
                                endpoint='/summon',
                                data={
                                    # The remote (summoned) drone handles logic to create a safety distance
                                    # offset between itself and the hotspot location.
                                    'lat': new_hotspot.lat,
                                    'lon': new_hotspot.lon,
                                    'airspeed': self.airspeed,
                                    'summon_drone_ip': self.this_drone[0],
                                    'summon_drone_port': self.this_drone[1]
                                },
                                skyserve_port=drone_port,
                                async=True,
                            )
            else:
                self.FLAG = -1
                self.log.warn('Failed to localize hotspot. Only has {} data points.'.format(len(self.wireless_data)))

    @callback(
        endpoint='/summon',
        description='Stop the current flight instruction and navigate to the requested '
                    'location.',
        required_params=('lat', 'lon', 'summon_drone_ip', 'summon_drone_ip', 'airspeed'),
        public=False,
    )
    def summon(self, data, *args, **kwargs):
        """
        Move to a remotely specified location.
        """
        self.should_avoid_summoning.value = True
        self.summoner_drone = (data['summon_drone_ip'], data['summon_drone_port'])
        self.log.debug('Summon drone is {summoner}'.format(summoner=self.summoner_drone))


        lat = data['lat']
        lon = data['lon']
        alt = self.swarm_altitude

        self.log.info('Received summon request to location: ({lat}, {lon}, {alt})'.format(
            lat=lat,
            lon=lon,
            alt=alt,
        ))

        # Abort whatever survey is currently in progress.
        self.should_abort_survey.value = True

        local_location = self.dc.read_gps()
        summon_dist = Coordinate.from_gps_data(local_location).distance_to(Coordinate(lat, lon))
        if summon_dist > MAX_TOLERABLE_REGION_SEPARATION_DISTANCE:
            self.log.debug('Too far.')
            return

        hotspot_coord = Coordinate(lat, lon)

        if not self.should_use_formation:
            # For safety, go to the requested location, but with a small offset of SUMMON_OFFSET meters.
            # Create a SUMMON_OFFSET meter offset from the target location, in the direction of the local drone.
            travel_coord = Coordinate(lat, lon).offset_toward_target(
                target=Coordinate.from_gps_data(self.dc.read_gps()),
                offset=SUMMON_OFFSET,
            )
            self.swarm_formation_offset = FormationOffset(
                bearing=hotspot_coord.bearing_toward(travel_coord),
                distance=hotspot_coord.distance_to(travel_coord),
            )

        # self.log.debug('Traveling to {travel_coord}'.format(travel_coord=travel_coord))
        try:
            self.visit_with_offset(
                coordinate=hotspot_coord,
                altitude=alt,
                speed=self.airspeed
            )
            self.log.debug('Drone arrived at estimated hotspot.')
        except AvionicsException:
            self.log.error('Drone reported inability to travel to summon coordinate!')
        except FlightAbortedException:
            self.log.error('Summon coordinate navigation aborted!')
        except RicePublicRelationsException:
            self.log.error('Summon coordinate {travel_coord} is invalid!'.format(
                travel_coord=travel_coord,
            ))
        self.send_completion_notification()
        self.log.debug('Navigation to summon location complete.')

        time.sleep(20)
        self.log.debug('Mission complete, landing!')
        self.dc.land()

        # if self.this_drone == self.summoner_drone:
        #     self.log.info('Local drone beginning shared wireless data')
        #     self.FLAG = 1
        #     self.is_summoner = True
        #     self.swarm_track()

    @tick(interval=1)
    def swarm_track(self, *args, **kwargs):
        """
        Combine the wireless data from another node in the system with that tracked locally.
        """
        if len(self.completion_messages) >= len(self.neighbors):
            self.log.debug('Other drones finished swarming. Attempting to calculate new hotspot')
        else:
            self.log.debug('Only {drones} finished traveling to hotspot. Will not attempt to calculate another hotspot yet.'.format(drones=self.completion_messages))
            return

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

            alphas.append(local_alpha)
            epsilons.append(local_epsilon)
            samples = [[sample['location']['lat'],
                        sample['location']['lon'],
                        sample['location']['alt'],
                        sample['wireless']['dBm']] for sample in samples]

            shared_data.append(samples)

        average_alpha = np.mean(alphas)
        average_epsilon = np.mean(epsilons)
        drone_count = len(self.neighbors)

        prediction = predict(dronenum=drone_count,
                             maxRun=1,
                             numIterations=GDParameters.NUM_ITERATIONS,
                             numEpoch=GDParameters.NUM_EPOCH,
                             threshold=GDParameters.THRESHOLD,
                             learning_rate=GDParameters.LEARNING_RATE,
                             numberBatch=1,
                             data_length=SAMPLES_SWARM)

        if 1 < drone_count <= 3:
            try:
                start = time.time()
                hotspot = prediction.swarm(drone_data=shared_data,
                                           alpha=average_alpha,
                                           epsilon=average_epsilon)
                end = time.time()
                self.swarmtime = end - start
                self.log.debug('Drone is getting data from {a} drones'.format(a=drone_count))
            except IndexError:
                self.log.warn('Drone did not get data good enough')
                return
        else:
            self.log.warn('Drone Number Incorrect')
            return

        self.log.debug('Calculated new hotspot at location: {}'.format(hotspot))
        combo_hotspot = Coordinate(hotspot[0], hotspot[1])

        if not self.region.contains(combo_hotspot) and not IS_SIMULATION:
            self.log.debug('New hotspot is out of region')
            return

        self.completion_messages = set()    # Reset the completion messages

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


        return GPSData(
            lat=hotspot_location[0],
            lon=hotspot_location[1],
            alt=hotspot_location[2],
        )

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

        # make the coordinates for the traversal points of rotation
        traverse_coords = [closest_corner_coord, second_corner_coord]
        shortest_side_dist = min([closest_corner_coord.distance_to(corner_coords[sides['short'][closest_corner]]),
                                  second_corner_coord.distance_to(corner_coords[sides['short'][second_corner]])])
        num_of_sweeps = int(round(shortest_side_dist / 10))
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
            self.log.debug('New traverse coordinates: {a}, {b} on iter {i}'.format(a=new_coord1, b=new_coord2, i=i))
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

    def visit_with_offset(self, coordinate, altitude, speed):
        """
        Fly to a coordinate moved by the given offset from center
        :param coordinate: the coordinate to fly to
        :param altitude: the altitude to fly to
        :param speed: the speed to fly the mission at
        """
        coordinate = self.swarm_formation_offset.offset_coordinate(coordinate)

        self.log.debug('Travelling to: {coord}'.format(coord=coordinate))
        self.dc.goto(coords=(coordinate.lat, coordinate.lon), altitude=altitude, airspeed=speed)

        location = self.dc.read_gps()
        self.log.debug('Arrived! Current location: ({lat}, {lon})'.format(
            lat=location.lat,
            lon=location.lon,
        ))

    def send_completion_notification(self):
        """
        Tells the summoner drone that the drone has finished its current task
        """
        msg_data={'ip_addr': self.this_drone[0], 'id': self.drone_id}
        drone_ip, drone_port = self.summoner_drone
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
        # cheater_speed = 0 # m/s
        # cheater_bearing = 270.0 # degrees: 0=North, 90=East, 180=South, 270=West
        cheater_height = 1.0 # meters

        # elapsed_time = time.time() - self.cheater_start_time
        # # Don't move the cheater until the DELAY has passed
        # if elapsed_time < 0:
        #     elapsed_time = 0
        #
        # self.hotspot_meters_moved = elapsed_time * cheater_speed

        current_local = self.dc.read_gps()
        current_pos = Coordinate.from_gps_data(current_local)

        # self.current_simulated_hotspot = self.initial_simulated_hotspot.offset_bearing(cheater_bearing, self.hotspot_meters_moved)

        horizontal_distance = current_pos.distance_to(Coordinate(TARGET_LOCATION[0],TARGET_LOCATION[1]))
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
