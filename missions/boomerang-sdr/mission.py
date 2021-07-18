import argparse
import time
import numpy as np

from rtlsdr import RtlSdr
from skyengine.drone import DroneController
from skyengine.exceptions import FlightAbortedException
from skylog.logger import DirectLogger
from skylog.message import BaseMessage
from skymission.concurrency import tick
from skymission.mission import Mission
from skymission.mission import callback
from skymission.mission import panic
from constants import REGIONS
from constants import GROUNDTRUTH_ALPHA
from constants import GROUNDTRUTH_EPSILON
import geo
from geo import Coordinate

from sensing_message import SensingMessage
from simulation import IS_SIMULATION


if IS_SIMULATION:
    import random
    from simulation import TARGET_LOCATION
else:
    from rtlsdr import RtlSdr


class Waypoint:
    def __init__(self, lat, lon):
        self.lat = lat
        self.lon = lon

class BoomerangMission(Mission):

    port = 4002
    mission_id = 'boomerang-sdr'

    def __init__(self, fc_addr):
        """
        Create a BoomerangMission and start the mission server.

        :param fc_addr: MAVProxy address of the flight controller.
        :param log_file: Name of the log file for location data.
        """
        self.enable_disk_event_logging()

        self.sensed_data = []

        self.dc = DroneController(fc_addr)
        self.log.debug('Drone controller and logger initialized successfully')

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
        self.sensor_range = 15 #in meters
        self.alpha = None
        self.epsilon = None
        self.dBm = None

        self.log.info('Boomerang-SDR mission initialization complete')
        self.start_server()

    @callback(
        endpoint='/start-mission',
        description='Gives the drone a series of parameters and starts the mission.',
        required_params=('waypoints', 'hover_duration', 'air_speed', 'altitudes', 'loops'),
        public=True,
    )

    def start_mission(self, data, *args, **kwargs):
 
        try:
            hover_duration = data['hover_duration']
            altitudes = data['altitudes']
            loops = data['loops']
            air_speed = data['air_speed']
            waypoints = [
                Waypoint(point['lat'], point['lon'])
                for point in data['waypoints']
            ]
            start_alt = altitudes[0]
            count_zero = False

            #Checking if the drone is in known regions and the 
            # waypoints are in the same region (safety reasons)
            current_region = False
            current_local = self.dc.read_gps()
            for i in REGIONS.keys():
                if(REGIONS[i].contains(current_local)):
                    current_region = REGIONS[i]
                    break
            if (current_region == False):
                self.log.debug('Drone out of known regions')
                return
            for i in range(0, len(waypoints)):

                if (not current_region.contains(waypoints[i])):
                    self.log.debug('Waypoint out of the current region')
                    return

            self.log.debug('Taking off to altitude: {alt}'.format(alt=start_alt))
            self.dc.take_off(start_alt)
            self.log.debug('Take off complete and starting wireless sensing at speed {speed}m/s'.format(speed=air_speed))
            self.wireless_sensing()
            
            if (loops == 0):
            	count_zero = True
            	loops = 1

            for i in range(0, len(altitudes)):
	            for j in range(0, loops):
	                for waypoint in waypoints:
	                    self.log.debug('Navigating to waypoint: ({lat}, {lon})'.format(
	                        lat=waypoint.lat,
	                        lon=waypoint.lon,
	                    ))
	                    self.dc.goto(coords=(waypoint.lat, waypoint.lon), altitude=altitudes[i], airspeed= data['air_speed'])

	                    location = self.dc.read_gps()
	                    self.log.debug('Arrived! Current location: ({lat}, {lon})'.format(
	                        lat=location.lat,
	                        lon=location.lon,
	                    ))

	                    self.log.debug('Hovering for {hover_duration} seconds'.format(
	                        hover_duration=hover_duration,
	                    ))
	                    time.sleep(hover_duration)

                    self.log.debug('Loop {j} at altitude {alt} complete'.format(
                    	j=(j+1),
                    	alt = altitudes[i],
                    ))

            if (not count_zero):
				self.dc.goto(coords=(waypoints[0].lat, waypoints[0].lon), altitude=altitudes[i], airspeed= data['air_speed'])

            self.log.info('Navigation to all waypoints complete. Landing now.')
            self.dc.land()
            self.log.info('Landed!')
        except FlightAbortedException:
            self.log.error('Flight aborted due to panic; aborting remaining tasks.')

    @panic
    def panic(self, *args, **kwargs):
        self.log.info('Mission panicked! Landing immediately.')
        self.dc.panic()

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

    def get_simulated_dBm(self):
        """
        Return a simulated dBm value that takes into account a moving target
        """
        target_height = 1.0 # meters

        current_local = self.dc.read_gps()
        current_pos = Coordinate.from_gps_data(current_local)

        horizontal_distance = current_pos.distance_to(Coordinate(TARGET_LOCATION[0],TARGET_LOCATION[1]))
        vertical_distance = current_local.alt - target_height
        distance_to_target = np.sqrt(vertical_distance**2 + horizontal_distance**2)

        dBm = GROUNDTRUTH_ALPHA * np.log10(distance_to_target) + GROUNDTRUTH_EPSILON

        dBm += np.random.normal(0,2) # Add random noise to the data

        return dBm


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--fc-addr',
        dest='fc_addr',
        help='Address of the flight controller mavproxy messages',
        default=None,
    )
    args = parser.parse_args()

    BoomerangMission(
        fc_addr=args.fc_addr,
    )
