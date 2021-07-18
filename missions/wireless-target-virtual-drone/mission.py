import multiprocessing
import time
import argparse
import utm
from math import sqrt
import json
import numpy as np

from skyengine.drone import DroneController
from skyengine.exceptions import AvionicsException
from skyengine.exceptions import FlightAbortedException
from skyengine.exceptions import RicePublicRelationsException
from skyengine.gpsdata import GPSData
from skylog.logger import DirectLogger
from skymission.mission import Mission
from skymission.mission import callback
from skymission.mission import panic
from skymission.concurrency import async_process

from gradient_descent import GPS2UTM_raw, RSSI_Loc_BatchedGD_5par
from kalman_filter import KalmanFilter
from swarm import predict
from astro_util import IntersectPoints

from constants import REGIONS
from constants import SIMULATED_ALPHA
from constants import SIMULATED_EPSILON
from constants import IS_ALPHA_CONTINUOUS

from geo import Coordinate
from hotspot_message import HotspotWirelessMessage #riccardo change to more happealing name

from simulation import IS_SIMULATION

if IS_SIMULATION:
    import random
else:
    from rtlsdr import RtlSdr

NUMBER_SAMPLES = 150
DISTANCE_TO_TARGET = 10

# The maximum tolerable number of meters away which the drone can be from the center of the
# requested survey region. This exists as a safety check in case the drone believes it is much
# farther away from the survey region than it actually is.
MAX_TOLERABLE_REGION_SEPARATION_DISTANCE = 150
if IS_SIMULATION:
    MAX_TOLERABLE_REGION_SEPARATION_DISTANCE = 500

# Delay before the simulated target moves. This allows the drones to take off before it starts moving.
MOVE_TARGET_DELAY = 30.0

class GDParameters(object):
    """
    Parameters used as input to the target localization algorithm.
    """
    NUM_ITERATIONS = 50
    NUM_EPOCH = 500
    THRESHOLD = 0.1
    LEARNING_RATE = 0.001


def get_triangle_coord(vertex1, target):
    '''
    Given two points (in coordinates), calculate the intersection between the circles around them with the radius:
    (distance between them)*sqrt(3), distance between them respectively. - Equilateral triangle
    '''
    triangle_size = vertex1.distance_to(target) * sqrt(3)

    [vx1, vy1, vz1, vzone_number, vzone_letter] = GPS2UTM_raw(vertex1.lat, vertex1.lon, 1)
    [hx1, hy1, hz1, hzone_number, hzone_letter] = GPS2UTM_raw(target.lat, target.lon, 1)

    vertex2, vertex3 = IntersectPoints(complex(vx1, vy1), complex(hx1,hy1), triangle_size, vertex1.distance_to(target))

    point2 = utm.to_latlon(vertex2.real, vertex2.imag, vzone_number, vzone_letter)
    point3 = utm.to_latlon(vertex3.real, vertex3.imag, hzone_number, hzone_letter)

    return point2, point3


class VirtualDroneMission(Mission):
    """
    Survey a defined quadrilateral region, looking for wireless targets. Once a drone is able to
    consistently identify a target, it will summon all other drones in the system to its location.
    Then the drones will begin to share data and relocalize the spectrum target, moving in a formation.
    """
    port = 4000
    mission_id = 'virtual-drone'

    def __init__(self, fc_addr):
        """
        Mission initialization procedure.

        :param fc_addr: Flight controller address.
        """
        self.enable_disk_event_logging()

        self.log.debug('Initializing mission...')

        self.wireless_data = []
        self.airspeed = 0
        self.region = None
        self.altitude = 0

        self.dc = DroneController(fc_addr)
        self.flight_logger = DirectLogger(path='{year}_{month}_{day}_{hour}_{min}_{sec}-virtual_drone.log'.format(
                                                year=time.localtime()[0],
                                                month=time.localtime()[1],
                                                day=time.localtime()[2],
                                                hour=time.localtime()[3],
                                                min=time.localtime()[4],
                                                sec=time.localtime()[5],
                                          ))

        if IS_SIMULATION:
            self.log.warn('Running mission as simulation. SDR will not be used')
            self.target_start_time = 0 ###
            self.initial_simulated_target = Coordinate(29.716491, -95.409222) #target POSITION IN THE CENTER OF STADIUM 0
            self.current_simulated_target = None
            self.target_meters_moved = 0
        else:
            self.sdr = RtlSdr()

            self.log.debug('Configuring RTL-SDR sensor.')
            self.sdr.sample_rate = 2.048e6  # Hz
            self.sdr.center_freq = 563e6     # Hz
            self.freq_correction = 60   # PPM
            self.sdr.gain = 10 # Supported gain values (29): 0.0 0.9 1.4 2.7 3.7 7.7 8.7 12.5 14.4 15.7 16.6 19.7 20.7 22.9 25.4 28.0 29.7 32.8 33.8 36.4 37.2 38.6 40.2 42.1 43.4 43.9 44.5 48.0 49.6

        self.sdr_samples_to_discard = 20
        self.sdr_number_sampled = 0
        self.number_sampling_points = 3

        self.alpha = None
        self.epsilon = None
        self.dBm = None
        self.target_location = None
        self.triangle_count = 0
        self.vertex = 0

        self.log.info('Mission initialization complete!')


    @callback(
        endpoint='/start-mission',
        description='Start the mission: assign a survey region for all drones, take off the '
                    'local drone, and request take off of all neighbor drones',
        required_params=('region_id', 'alt', 'airspeed'),
        public=True,
    )
    def start_mission(self, data, *args, **kwargs):
        """
        The input parameters are:

            - region_id (str): One of the region IDs defined in the constant REGIONS
            - alt (int): The survey altitude
            - airspeed (int): Airspeed of the drone
        """
        self.airspeed = data['airspeed']
        self.altitude = data['alt']
        self.region = REGIONS[data['region_id']]

        start_time = time.time() + MOVE_TARGET_DELAY

        values={
                'start_time': start_time,
            }

        self.run_mission(values)


    def run_mission(self, data, *args, **kwargs):

        self.target_start_time = data['start_time']

        current_local = self.dc.read_gps()
        self.log.debug('Taking off: altitude {alt}'.format(alt=self.altitude))
        self.dc.take_off(self.altitude)

        ## IF WANT TO FIX ALPHA AND EPSILON, COMMENT 2 LINES BELOW AND WRITE MANUALLY SELF.ALPHA = ... ETC
        self.wireless_sensing(NUMBER_SAMPLES)
        self.learn_enviroment()
        self.smart_pattern(self.target_location)

    def wireless_sensing(self, num_Samples):
        """
        Perform a scan with the wireless sensor, updating internal mission state with
        the sensed data values.
        """
        cont_Samples = 0
        while cont_Samples <= num_Samples:

            gps_pos = self.dc.read_gps()
            n_samples = 256  # DON'T CHANGE TO 128!!!!! IT CAUSES KERNEL PANIC (unless you change tick or find another fix)

            if IS_SIMULATION:
                dBm = self.get_simulated_dBm()
                self.dBm = dBm
                self.sdr_number_sampled += 1
                time.sleep(0.01)
            else:
                self.sdr_number_sampled += 1
                samples = self.sdr.read_samples(n_samples)
                dBm = 10 * np.log10(np.mean(np.power(np.abs(samples), 2)))
                self.dBm = dBm

            if self.sdr_number_sampled > self.sdr_samples_to_discard:
                wireless_msg0 = HotspotWirelessMessage(
                	triangle_count=self.triangle_count,
                	vertex=self.vertex,
                    location=gps_pos,
                    heading=self.dc.vehicle.heading,
                    dBm=dBm,
                )
                self.flight_logger.log(wireless_msg0)
                self.wireless_data.append(wireless_msg0)
                cont_Samples += 1

    def learn_enviroment(self):
    	'''
		Calculate values of alpha and epsilon based on the last NUMBER_SAMPLES which
		were measured by the wireless_sensing function
		'''
        samples = np.zeros(shape=(NUMBER_SAMPLES, 4))
        len_data = len(self.wireless_data)

        samples = self.filter_data()
        samples = [[sample['lat'],
                    sample['lon'],
                    sample['alt'],
                    sample['dBm']] for sample in json.loads(samples)]

        self.log.debug('Calculating alpha and epsilon with last data points')

        _, [self.alpha, self.epsilon] = RSSI_Loc_BatchedGD_5par(
            data=samples,
            numEpoch=GDParameters.NUM_EPOCH,
            numIterations=GDParameters.NUM_ITERATIONS,
            threshold=GDParameters.THRESHOLD,
            learning_rate=GDParameters.LEARNING_RATE,
        )

        self.log.debug('Alpha: {a} Epsilon: {e}'.format(a=self.alpha, e=self.epsilon))

    def smart_pattern(self, target):
        '''
        Calculate 3 positions around a new target, move the drone through them and collect/store data
        to calculate a new target
        '''
        alphas = []
        epsilons = []
        positions = []
        data_collected = []

        # if target == None:
        #     self.log.debug('Calculating emulated target')
        #     samples = self.filter_data()

        #     alphas.append(self.alpha)
        #     epsilons.append(self.epsilon)

        #     samples = [[sample['lat'],
        #                 sample['lon'],
        #                 sample['alt'],
        #                 sample['dBm']] for sample in json.loads(samples)]

        #     data_collected.append(samples)
        #     self.target_location = self.calculate_target_location(alphas, epsilons, data_collected)
        #     return self.smart_pattern(self.target_location)

        # else:
        self.triangle_count += 1
        self.log.debug('Beginning smart_pattern')

        triangle_positions = self.calculate_vertices(target, DISTANCE_TO_TARGET)

        for i in range(0, len(triangle_positions)):
            self.log.debug('Flying to waypoint: {coord}'.format(coord=triangle_positions[i]))

            self.dc.goto(coords=(triangle_positions[i].lat, triangle_positions[i].lon), altitude=self.altitude, airspeed=self.airspeed)
            self.log.debug('Arrived, start the sensing')

            self.vertex = (self.vertex%3) + 1
            self.wireless_sensing(NUMBER_SAMPLES)

            if IS_ALPHA_CONTINUOUS:
            	self.learn_enviroment()
            	alphas.append(self.alpha)
            	epsilons.append(self.epsilon)

            else:
            	alphas.append(self.alpha)
            	epsilons.append(self.epsilon)

            samples = self.filter_data()

            samples = [[sample['lat'],
                        sample['lon'],
                        sample['alt'],
                        sample['dBm']] for sample in json.loads(samples)]

            data_collected.append(samples)

            self.log.debug('Sensing done')

        self.target_location = self.calculate_target_location(alphas, epsilons, data_collected)

        return self.smart_pattern(self.target_location)

    def calculate_vertices(self, target, dist_to_target):

        vertices = []
        local_location = self.dc.read_gps()
        local_coord = Coordinate(local_location.lat, local_location.lon)


        if target == None:
            vertices.append(local_coord)
            emulated_target = local_coord.offset_toward_target(self.region.center(), dist_to_target)
            aux1, aux2 = get_triangle_coord(vertices[0], emulated_target)
        elif local_coord.distance_to(target) > dist_to_target:
            distaux = local_coord.distance_to(target) - dist_to_target
            vertices.append(local_coord.offset_toward_target(target, distaux))
            aux1, aux2 = get_triangle_coord(vertices[0], target)
        else:
            distaux = dist_to_target - local_coord.distance_to(target)
            vertices.append(target.offset_toward_target(local_coord, distaux))
            aux1, aux2 = get_triangle_coord(vertices[0], target)

        vertices.append(Coordinate(aux1[0], aux1[1]))
        vertices.append(Coordinate(aux2[0], aux2[1]))

        #if not IS_SIMULATION:
        for i in range(0, len(vertices)):
            if not self.region.contains(vertices[i]):
            	self.log.debug('Vertices out of region. Retrying with dist_to_tx = {}'.format(dist_to_target-1))
            	return self.calculate_vertices(target, (dist_to_target-1))

        return vertices

    def filter_data(self):
    	'''
		Return the values of position and dBm sensed by the drone on the wireless_sensing function,
		using the Kalman Filter.
    	'''
        kf = KalmanFilter(0.008, 10)
        samples = np.zeros(shape=(NUMBER_SAMPLES, 4))
        len_data = len(self.wireless_data)

        for idx, sample in enumerate(self.wireless_data[len_data-NUMBER_SAMPLES:len_data]):
            samples[idx][0] = sample.lat
            samples[idx][1] = sample.lon
            samples[idx][2] = sample.alt
            samples[idx][3] = sample.dBm

        data_kf_filter = json.dumps([{
        'lat': i[0],
        'lon': i[1],
        'alt': i[2],
        'dBm': kf.filter(i[3]),
        } for i in samples])

        return data_kf_filter


    def calculate_target_location(self, alphas, epsilons, data_collected):
        """
        Combine the wireless data in the system to calculate a new target
        """
        if len(alphas) == 1:
        	for i in range(0, self.number_sampling_points-1):
        		alphas.append(alphas[0])
        		epsilons.append(epsilons[0])

        # if self.target_location == None:
        #     # dBm_list = []
        #     # for sample in data_collected[0][3]:
        #     #     dBm_list.append(sample)

        #     # average_dBm = sum(dBm_list) / float(len(dBm_list))
        #     # radius_target_position = 10 ** ((average_dBm - self.epsilon) / self.alpha)
        #     # ###TODO: fix radius_target_position
        #     # if radius_target_position > self.altitude:
        #     #     horizontal_distance = sqrt((radius_target_position**2) - (self.altitude**2))
        #     # else:
        #     #     horizontal_distance = 0

        #     local_position = self.dc.read_gps()
        #     local_coord = Coordinate(local_position.lat, local_position.lon)

        #     first_emulated_target = local_coord.offset_toward_target(self.region.center(), DISTANCE_TO_TARGET)

        #     self.log.debug('=========================================================================')
        #     self.log.debug('Calculated emulated target at location: {}'.format(first_emulated_target))
        #     self.log.debug('=========================================================================')

        #     return first_emulated_target

        # else:
        prediction = predict(dronenum=self.number_sampling_points,
                             maxRun=1,
                             numIterations=GDParameters.NUM_ITERATIONS,
                             numEpoch=GDParameters.NUM_EPOCH,
                             threshold=GDParameters.THRESHOLD,
                             learning_rate=GDParameters.LEARNING_RATE,
                             numberBatch=1,
                             data_length=NUMBER_SAMPLES*self.number_sampling_points)

        try:
            target = prediction.swarm(drone_data=data_collected,
                                       alphas=alphas,
                                       epsilons=epsilons)
        except IndexError:
            self.log.warn('Target localization failed. Data not good enough.')
            return False

        computed_target_position = Coordinate(target[0], target[1])

        self.log.debug('=========================================================================')
        self.log.debug('Calculated new target at location: {}'.format(computed_target_position))

        if IS_SIMULATION:
            error = computed_target_position.distance_to(self.current_simulated_target)
            self.log.debug('Simulated error: {err}, Simulated target has moved {dist} meters to: {loc}'.format(
                    err=error,
                    dist=self.target_meters_moved,
                    loc=self.current_simulated_target
                ))
        self.log.debug('=========================================================================')

        if not self.region.contains(computed_target_position) and not IS_SIMULATION:
            self.log.debug('New target is out of region')
            self.log.debug('Setting new target location as the latest one calculated')
            return self.target_location

        return computed_target_position


    @panic
    def panic(self, *args, **kwargs):
        """
        This mission's panic behavior delegates to Skyengine.
        """
        self.log.error('Panicking drone!')
        self.dc.panic()


    def get_simulated_dBm(self):
        """
        Return a simulated dBm value that takes into account a moving target
        """
        target_speed = 0 # m/s
        target_bearing = 90.0 # degrees: 0=North, 90=East, 180=South, 270=West
        target_height = 1.0 # meters

        elapsed_time = time.time() - self.target_start_time
        # Don't move the target until the DELAY has passed
        if elapsed_time < 0:
            elapsed_time = 0

        self.target_meters_moved = elapsed_time * target_speed

        current_local = self.dc.read_gps()
        current_pos = Coordinate.from_gps_data(current_local)

        self.current_simulated_target = self.initial_simulated_target.offset_bearing(target_bearing, self.target_meters_moved)
        horizontal_distance = current_pos.distance_to(self.current_simulated_target)
        vertical_distance = current_local.alt - target_height
        distance_to_target = sqrt(vertical_distance**2 + horizontal_distance**2)

        dBm = SIMULATED_ALPHA * np.log10(distance_to_target) + SIMULATED_EPSILON
        dBm += np.random.normal(0,2) # Add random noise to data

        return dBm


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--fc-addr',
        dest='fc_addr',
        help='Address of the flight controller mavproxy messages',
    )
    args = parser.parse_args()

    mission = VirtualDroneMission(fc_addr=args.fc_addr)
    mission.start_server()
