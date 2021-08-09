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
#import geo 

class Waypoint:
    def __init__(self, lat, lon, alt):
        self.lat = lat
        self.lon = lon
        self.alt = alt

class LocationMessage(BaseMessage):
    """
    Message for the drone's current location.
    """

    def __init__(self, timestamp, lat, lon, alt):
        self.timestamp = timestamp
        self.lat = lat
        self.lon = lon
        self.alt = alt

    def serialize(self):
        return {
            'timestamp': self.timestamp,
            'lat': self.lat,
            'lon': self.lon,
            'alt': self.alt,
        }

    @staticmethod
    def deserialize(json):
        return LocationMessage(
            timestamp=json['timestamp'],
            lat=json['lat'],
            lon=json['lon'],
            alt=json['alt'],
        )


class SimpleTestMission(Mission):
    """
    A mission to take off, move forward horizontally while moving up and down vertically.
    """

    port = 4000
    mission_id = 'simple_test'

    def __init__(self, fc_addr, log_file):
        """
        Create a SimpleTestMission, which is started as soon as the class is instantiated.

        :param fc_addr: MAVProxy address of the flight controller.
        :param log_file: Name of log file for location data.
        """
        self.enable_disk_event_logging()

        self.dc = DroneController(fc_addr)
        self.location_logger = DirectLogger(path=log_file)
        self.log.debug('Drone controller and logger initialized successfully')

        self.cancel_tick = self.start_location_log()
        self.log.info('Simple_test mission initialization complete')

        self.start_server()

    @tick(interval=0.5)
    def start_location_log(self):
        """
        Start periodically logging the drone GPS location to disk.
        """
        location = self.dc.vehicle.location.global_frame
        message = LocationMessage(
            timestamp=time.time(),
            lat=location.lat,
            lon=location.lon,
            alt=location.alt,
        )

        # self.log.debug('Recording current location: ({lat}, {lon})'.format(
        #     lat=location.lat,
        #     lon=location.lon,
        # ))

        self.location_logger.log(message)

    @callback(
        endpoint='/start-mission',
        description='Gives the drone an altitude and distance.',
        required_params=('alt'),
        public=True,
    )
    def start_mission(self, data, *args, **kwargs):
        """
        Client-invoked endpoint to begin the hori_vert mission.

        :param data: Required to be of the form:
                     {
                         'alt': ...,  # Target altitude (m)
                         'distance': ..., # distance travel (s)
                     }
        """
        alt = data['alt']


        #Initializing our coordinates, which should be 3 meters from each other 
        
        ####For future reference, consider making the number of up-and-down vertical movements a parameter####
        #Start location
        lat_start = 29.716334
        lon_start = -95.409326
        #End Location
        lat_end = 29.716294
        lon_end = -95.409326

        start_coord = Waypoint(lat_start, lon_start, alt)
        end_coord = Waypoint(lat_end, lon_end, alt)


        try:
            self.log.debug('Taking off to altitude: {alt}'.format(alt=alt))

            location = self.dc.vehicle.location.global_frame

            self.dc.take_off(alt)
            self.log.debug('Reached altitude, moving to start location of {alt}'.format(alt=alt))

            # self.log.debug('Hovering for: {hover_time} seconds'.format(hover_time=hover_time))
            # time.sleep(hover_time)

            # self.log.info('Hovering complete; moving to start')
            # self.dc.land()
            # self.log.info('Landed!')

            location = self.dc.vehicle.location.global_frame
            self.dc.goto(coords=(start_coord.lat, start_coord.lon), altitude=start_coord.alt, airspeed=2)
            self.log.debug('Arrived at starting location ({lat}, {lon}), now heading toward end location'.format(lat=lat_start, lon=lon_start))
            time.sleep(5)


            location = self.dc.vehicle.location.global_frame
            self.dc.goto(coords=(end_coord.lat, end_coord.lon), altitude=end_coord.alt, airspeed=2)
            self.log.debug('Arrived at end location ({lat}, {lon}). Landing'.format(lat=lat_end, lon=lon_end))
            time.sleep(5)


            self.dc.land()
            self.log.info('Landed!')

        except FlightAbortedException:
            self.log.warn('Flight aborted due to emergency panic!')

    @panic
    def panic(self, *args, **kwargs):
        self.log.warn('Panic request received')
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
        default='travel-path',
    )
    args = parser.parse_args()

    SimpleTestMission(
        fc_addr=args.fc_addr,
        log_file=args.log_file,
    )
