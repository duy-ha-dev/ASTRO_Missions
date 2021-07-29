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


class VerticalMission(Mission):
    """
    A mission to hover a certain altitude, then move up to another altitude.
    """

    port = 4000
    mission_id = 'vertical'

    def __init__(self, fc_addr, log_file):
        """
        Create a VerticalMission, which is started as soon as the class is instantiated.

        :param fc_addr: MAVProxy address of the flight controller.
        :param log_file: Name of log file for location data.
        """
        self.enable_disk_event_logging()

        self.dc = DroneController(fc_addr)
        self.location_logger = DirectLogger(path=log_file)
        self.log.debug('Drone controller and logger initialized successfully')

        self.cancel_tick = self.start_location_log()
        self.log.info('Vertical mission initialization complete')

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
        required_params=('alt1', 'alt2', 'position'),
        public=True,
    )
    def start_mission(self, data, *args, **kwargs):
        """
        Client-invoked endpoint to begin the vertical mission.

        :param data: Required to be of the form:
                     {
                         'alt': ...,  # Target altitude (m)
                         'distance': ..., # distance travel (s)
                     }
        """
        alt1 = data['alt1']
        alt2 = data['alt2']
        position = data['position']


        #Get coordinates from our position
        if position == 1:
            lat = 29.716170
            lon = 95.409253
        elif position == 2:
            lat = 29.716084
            lon = 95.409253
        elif position == 3:
            lat = 29.716006
            lon = 95.409253
        else:
            self.log.debug('Invalid position: please give a position 1, 2, or 3.')


        #Start and end coordinates
        start_coord = Waypoint(lat, lon, alt1)
        end_coord = Waypoint(lat, lon, alt2)

        try:
            self.log.debug('Taking off to first altitude: {alt1}'.format(alt=alt1))
            self.dc.take_off(alt1)
            self.log.debug('Reached altitude, moving to start location')

            # self.log.debug('Hovering for: {hover_time} seconds'.format(hover_time=hover_time))
            # time.sleep(hover_time)

            # self.log.info('Hovering complete; moving to start')
            # self.dc.land()
            # self.log.info('Landed!')

            self.dc.goto(coords=(start_coord.lat, start_coord.lon), altitude=start_coord.alt, airspeed=2)
            self.log.debug('Arrived at starting location, now heading toward end location in 10 seconds')
            time.sleep(10)

            self.dc.goto(coords=(end_coord.lat, end_coord.lon), altitude=end_coord.alt, airspeed=2)
            self.log.debug('Arrived at end location. Landing in 10 seconds')
            time.sleep(10)

            self.log.debug('Landing')
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

    VerticalMission(
        fc_addr=args.fc_addr,
        log_file=args.log_file,
    )
