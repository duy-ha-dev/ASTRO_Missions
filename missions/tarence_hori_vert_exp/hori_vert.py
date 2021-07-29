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


class HoriVertMission(Mission):
    """
    A mission to take off, move forward horizontally while moving up and down vertically.
    """

    port = 4000
    mission_id = 'hori_vert'

    def __init__(self, fc_addr, log_file):
        """
        Create a HoriVertMission, which is started as soon as the class is instantiated.

        :param fc_addr: MAVProxy address of the flight controller.
        :param log_file: Name of log file for location data.
        """
        self.enable_disk_event_logging()

        self.dc = DroneController(fc_addr)
        self.location_logger = DirectLogger(path=log_file)
        self.log.debug('Drone controller and logger initialized successfully')

        self.cancel_tick = self.start_location_log()
        self.log.info('Hori_Vert mission initialization complete')

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
        required_params=('alt1', 'alt2'),
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
        alt1 = data['alt1']
        alt2 = data['alt2']


        #Initializing our 5 points
        ####For future reference, consider making the number of up-and-down vertical movements a parameter####
        lat1 = 29.716335
        lon1 = 95.409253
        lat2 = 29.716253
        lon2 = 95.409253
        lat3 = 29.716170
        lon3 = 95.409253
        lat4 = 29.716084
        lon4 = 95.409253
        lat5 = 29.716006
        lon5 = 95.409253

        first_coord = Waypoint(lat1, lon1, alt1)
        second_coord = Waypoint(lat2, lon2, alt2)
        third_coord = Waypoint(lat3, lon3, alt1)
        fourth_coord = Waypoint(lat4, lon4, alt2)
        fifth_coord = Waypoint(lat5, lon5, alt1)


        try:
            self.log.debug('Taking off to altitude: {alt}'.format(alt=alt1))
            self.dc.take_off(alt1)
            self.log.debug('Reached altitude, moving to start location')

            # self.log.debug('Hovering for: {hover_time} seconds'.format(hover_time=hover_time))
            # time.sleep(hover_time)

            # self.log.info('Hovering complete; moving to start')
            # self.dc.land()
            # self.log.info('Landed!')

            self.dc.goto(coords=(first_coord.lat, first_coord.lon), altitude=first_coord.alt, airspeed=2)
            self.log.debug('Arrived at starting location, now heading toward point 2')
            time.sleep(5)

            self.dc.goto(coords=(second_coord.lat, second_coord.lon), altitude=second_coord.alt, airspeed=2)
            self.log.debug('Arrived at point 2, now heading toward point 3')
            time.sleep(5)

            self.dc.goto(coords=(third_coord.lat, third_coord.lon), altitude=third_coord.alt, airspeed=2)
            self.log.debug('Arrived at point 3, now heading toward point 4')
            time.sleep(5)

            self.dc.goto(coords=(fourth_coord.lat, fourth_coord.lon), altitude=fourth_coord.alt, airspeed=2)
            self.log.debug('Arrived at point 4, now heading toward point 5')
            time.sleep(5)

            self.dc.goto(coords=(fifth_coord.lat, fifth_coord.lon), altitude=fifth_coord.alt, airspeed=2)
            self.log.debug('Arrived at point 5, now landing')
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

    HoriVertMission(
        fc_addr=args.fc_addr,
        log_file=args.log_file,
    )
