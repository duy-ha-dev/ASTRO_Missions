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

# class Waypoint:
#     def __init__(self, lat, lon, alt):
#         self.lat = lat
#         self.lon = lon
#         self.alt = alt

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


class AoAMission(Mission):
    """
    A mission to take off, move toward 6 specified points along an arc, and land.
    """

    port = 4000
    mission_id = 'aoa'

    def __init__(self, fc_addr, log_file):
        """
        Create a AoAMission, which is started as soon as the class is instantiated.

        :param fc_addr: MAVProxy address of the flight controller.
        :param log_file: Name of log file for location data.
        """
        self.enable_disk_event_logging()

        self.dc = DroneController(fc_addr)
        self.location_logger = DirectLogger(path=log_file)
        self.log.debug('Drone controller and logger initialized successfully')

        self.cancel_tick = self.start_location_log()
        self.log.info('AoA point-to-point mission initialization complete')

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
        Client-invoked endpoint to begin the AoA mission.

        :param data: Required to be of the form:
                     {
                         'alt': ...,  # Target altitude (m)
                         'distance': ..., # distance travel (s)
                     }
        """
        alt = data['alt']
  


        #Initializing our 5 points
        ####For future reference, consider making the number of up-and-down vertical movements a parameter####
        lat1 = 29.716639
        lon1 = -95.409102
        lat2 = 29.716492
        lon2 = -95.409221
        lat3 = 29.716332
        lon3 = -95.409306
        lat4 = 29.716177
        lon4 = -95.409357
        lat5 = 29.715990
        lon5 = -95.409374
        lat6 = 29.715821
        lon6 = -95.409368

        # first_coord = Waypoint(lat1, lon1, alt)
        # second_coord = Waypoint(lat2, lon2, alt)
        # third_coord = Waypoint(lat3, lon3, alt)
        # fourth_coord = Waypoint(lat4, lon4, alt)
        # fifth_coord = Waypoint(lat5, lon5, alt)
        # sixth_coord = Waypoint(lat6, lon6, alt)


        try:
            self.log.debug('Taking off to altitude: {alt}'.format(alt=alt))
            self.dc.take_off(alt)
            self.log.debug('Take off complete')

            time.sleep(3)

            location = self.dc.read_gps()
            self.log.debug('Current location: ({lat}, {lon})'.format(lat=location.lat,lon=location.lon,))

            # self.log.debug('Hovering for: {hover_time} seconds'.format(hover_time=hover_time))
            # time.sleep(hover_time)

            # self.log.info('Hovering complete; moving to start')
            # self.dc.land()
            # self.log.info('Landed!')

        #Move to First Location
            nextlat, nextlon = lat1, lon1

            self.log.debug('Navigating to waypoint 1: ({lat}, {lon})'.format(lat=nextlat,lon=nextlon,))
            self.dc.goto(coords=(nextlat, nextlon), altitude=alt, airspeed=2)
            self.log.debug('Navigation to waypoint complete')

            location = self.dc.read_gps()
            self.log.debug('Arrived! Current location: ({lat}, {lon})'.format(lat=location.lat,lon=location.lon,))

            time.sleep(5)

        #Move to Second Location
            nextlat, nextlon = lat2, lon2

            self.log.debug('Navigating to waypoint 2: ({lat}, {lon})'.format(lat=nextlat,lon=nextlon,))
            self.dc.goto(coords=(nextlat, nextlon), altitude=alt, airspeed=2)
            self.log.debug('Navigation to waypoint complete')

            location = self.dc.read_gps()
            self.log.debug('Arrived! Current location: ({lat}, {lon})'.format(lat=location.lat,lon=location.lon,))

            time.sleep(5)

        #Move to Third Location
            nextlat, nextlon = lat3, lon3

            self.log.debug('Navigating to waypoint 3: ({lat}, {lon})'.format(lat=nextlat,lon=nextlon,))
            self.dc.goto(coords=(nextlat, nextlon), altitude=alt, airspeed=2)
            self.log.debug('Navigation to waypoint complete')

            location = self.dc.read_gps()
            self.log.debug('Arrived! Current location: ({lat}, {lon})'.format(lat=location.lat,lon=location.lon,))

            time.sleep(5)
        
        #Move to Fourth Location
            nextlat, nextlon = lat4, lon4

            self.log.debug('Navigating to waypoint 4: ({lat}, {lon})'.format(lat=nextlat,lon=nextlon,))
            self.dc.goto(coords=(nextlat, nextlon), altitude=alt, airspeed=2)
            self.log.debug('Navigation to waypoint complete')

            location = self.dc.read_gps()
            self.log.debug('Arrived! Current location: ({lat}, {lon})'.format(lat=location.lat,lon=location.lon,))

            time.sleep(5)
        
        #Move to Fifth Location
            nextlat, nextlon = lat5, lon5

            self.log.debug('Navigating to waypoint 5: ({lat}, {lon})'.format(lat=nextlat,lon=nextlon,))
            self.dc.goto(coords=(nextlat, nextlon), altitude=alt, airspeed=2)
            self.log.debug('Navigation to waypoint complete')

            location = self.dc.read_gps()
            self.log.debug('Arrived! Current location: ({lat}, {lon})'.format(lat=location.lat,lon=location.lon,))

            time.sleep(5)

        #Move to Sixth Location
            nextlat, nextlon = lat6, lon6

            self.log.debug('Navigating to waypoint 6: ({lat}, {lon})'.format(lat=nextlat,lon=nextlon,))
            self.dc.goto(coords=(nextlat, nextlon), altitude=alt, airspeed=2)
            self.log.debug('Navigation to waypoint complete')

            location = self.dc.read_gps()
            self.log.debug('Arrived! Current location: ({lat}, {lon})'.format(lat=location.lat,lon=location.lon,))

            time.sleep(5)

            
            self.log.info('Mission complete; begin landing')
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

    AoAMission(
        fc_addr=args.fc_addr,
        log_file=args.log_file,
    )
