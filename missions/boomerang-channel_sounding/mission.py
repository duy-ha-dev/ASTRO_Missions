import argparse
import time

from skyengine.drone import DroneController
from skyengine.exceptions import FlightAbortedException
from skylog.logger import DirectLogger
from skylog.message import BaseMessage
from skymission.concurrency import tick
from skymission.concurrency import async_process
from skymission.mission import Mission
from skymission.mission import callback
from skymission.mission import panic

from constants import REGIONS
import geo


class Waypoint:
    def __init__(self, lat, lon):
        self.lat = lat
        self.lon = lon


class BoomerangMessage(BaseMessage):
    """
    Message for the drone's current location.
    """

    def __init__(self, timestamp, lat, lon, alt, heading, speed, roll, pitch, yaw):
        self.timestamp = timestamp
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.heading = heading
        self.speed = speed
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    def serialize(self):
        return {
            'timestamp': self.timestamp,
            'lat': self.lat,
            'lon': self.lon,
            'alt': self.alt,
            'heading': self.heading,
            'speed': self.speed,
            'roll': self.roll,
            'yaw': self.yaw,
            'pitch': self.pitch
        }

    @staticmethod
    def deserialize(json):
        return BoomerangMessage(
            timestamp=json['timestamp'],
            lat=json['lat'],
            lon=json['lon'],
            alt=json['alt'],
            speed=json['speed'],
            heading=json['heading'],
            roll=json['roll'],
            pitch=json['pitch'],
            yaw=json['yaw']
        )


class BoomerangMission(Mission):

    port = 4002
    mission_id = 'boomerang'

    def __init__(self, fc_addr, log_file):
        """
        Create a BoomerangMission and start the mission server.

        :param fc_addr: MAVProxy address of the flight controller.
        :param log_file: Name of the log file for location data.
        """
        self.enable_disk_event_logging()

        self.dc = DroneController(fc_addr)
        self.location_logger = DirectLogger(path=log_file)
        self.log.debug('Drone controller and logger initialized successfully')

        self.log.info('Boomerang mission initialization complete')

        self.start_server()

    @tick()
    def start_location_log(self):
        """
        Start periodically logging the drone GPS location to disk.
        """
        location = self.dc.read_gps()
        message = BoomerangMessage(
            timestamp=time.time(),
            lat=location.lat,
            lon=location.lon,
            alt=location.alt,
            speed=self.dc.vehicle.airspeed,
            heading=self.dc.vehicle.heading,
            roll=str(self.dc.vehicle.attitude.roll),
            pitch=str(self.dc.vehicle.attitude.pitch),
            yaw=str(self.dc.vehicle.attitude.yaw),
        )

        self.location_logger.log(message)

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
            self.log.debug('Take off complete')

            #start logging
            self.start_location_log()
            self.run_sdr_measurements()

            if (loops == 0):
                count_zero = True
                loops = 1

            for i in range(0, len(altitudes)):
                for j in range(0, loops):
                    for waypoint in waypoints:
                        self.log.debug('Navigating at {speed} to waypoint: ({lat}, {lon})'.format(
                            speed=air_speed,
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

    @async_process
    def run_sdr_measurements(self):
        '''
        Runs SDR scripts needed for channel sounding
        '''
        import subprocess
        subprocess.call("SDR/IRIS/ChannelSounding/IrisUtils_setupNB.py --serial RF3C000026", shell=True) ###FIX SCRITP IrisUtils_setupNB.py

    @panic
    def panic(self, *args, **kwargs):
        self.log.info('Mission panicked! Landing immediately.')
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
        default='{year}_{month}_{day}_{hour}_{min}_{sec}-boomerang.log'.format(
                                                year=time.localtime()[0],
                                                month=time.localtime()[1],
                                                day=time.localtime()[2],
                                                hour=time.localtime()[3],
                                                min=time.localtime()[4],
                                                sec=time.localtime()[5],
                                          ))
    args = parser.parse_args()

    BoomerangMission(
        fc_addr=args.fc_addr,
        log_file=args.log_file,
    )
