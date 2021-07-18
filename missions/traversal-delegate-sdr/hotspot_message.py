import time

from skyengine.gpsdata import GPSData
from skylog.message import BaseMessage


class HotspotWirelessMessage(BaseMessage):
    """
    Definition of the structured message to periodically log sensed wireless data to disk.
    """

    def __init__(self, location, dBm, sdr, heading):
        """
        Initialize a message.

        :param drone_id: ID of this drone, generated at runtime.
        :param location: Current location, as a GPSData object.
        :param dBm: dBm value at this frequency.\
        :param sdr: raw IQ samples
        """
        self.timestamp = time.time()
        self.lat = location.lat
        self.lon = location.lon
        self.alt = location.alt
        self.dBm = dBm
        self.sdr = sdr
        self.heading = heading

    def serialize(self):
        """
        Generate a string representation of this message for logging to disk.

        :return: JSON representation of the message.
        """
        return {
            'timestamp': self.timestamp,
            'location': {
                'lat': self.lat,
                'lon': self.lon,
                'alt': self.alt,
            },
            'wireless': {
                'dBm': self.dBm,
                'IQ': [],
            },
            'heading': self.heading
        }

    @staticmethod
    def deserialize(json):
        return HotspotWirelessMessage(
            location=GPSData(
                lat=json['location']['lat'],
                lon=json['location']['lon'],
                alt=json['location']['alt'],
            ),
            dBm=json['wireless']['dBm'],
            sdr=json['wireless']['IQ'],
        )


class HotspotFilterMessage(BaseMessage):
    """
    Definition of the structured message to periodically log sensed wireless data to disk.
    """

    def __init__(self, location, dBm, sdr, FLAG=0, hotcaltime=None, alepcaltime=None, survetime=None, swarmtime=None):
        """
        Initialize a message.

        :param drone_id: ID of this drone, generated at runtime.
        :param location: Current location, as a GPSData object.
        :param dBm: dBm value at this frequency.\
        :param sdr: raw IQ samples
        """
        self.timestamp = time.time()
        self.lat = location.lat
        self.lon = location.lon
        self.alt = location.alt
        self.dBm = dBm
        self.sdr = sdr
        self.hotcaltime = hotcaltime
        self.alepcaltime = alepcaltime
        self.survetime = survetime
        self.swarmtime = swarmtime
        self.FLAG = FLAG

    def serialize(self):
        """
        Generate a string representation of this message for logging to disk.

        :return: JSON representation of the message.
        """
        return {
            'timestamp': self.timestamp,
            'FLAG': self.FLAG,
            'hotspot_time': self.hotcaltime,
            'alep_time': self.alepcaltime,
            'survey_time': self.survetime,
            'location': {
                'lat': self.lat,
                'lon': self.lon,
                'alt': self.alt,
            },
            'wireless': {
                'dBm': self.dBm,
                'IQ': [],
            },
        }

    @staticmethod
    def deserialize(json):
        return HotspotWirelessMessage(
            location=GPSData(
                lat=json['location']['lat'],
                lon=json['location']['lon'],
                alt=json['location']['alt'],
            ),
            dBm=json['wireless']['dBm'],
            sdr=json['wireless']['IQ']
        )

class UsedData(BaseMessage):
    """
    Definition of the structured message to periodically log sensed wireless data to disk.
    """

    def __init__(self, used_data):
        """
        Initialize a message.
        """
        self.used_data = used_data

        # self.FLAG = FLAG

    def serialize(self):
        """
        Generate a string representation of this message for logging to disk.

        :return: JSON representation of the message.
        """
        return {
            'used_data': self.used_data,
        }

    @staticmethod
    def deserialize(json):
        return UsedData(
            used_data=json['used_data']
        )
