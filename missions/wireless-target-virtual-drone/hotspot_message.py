import time

from skyengine.gpsdata import GPSData
from skylog.message import BaseMessage


class HotspotWirelessMessage(BaseMessage):
    """
    Definition of the structured message to periodically log sensed wireless data to disk.
    """

    def __init__(self, triangle_count, vertex, location, heading, dBm):
        """
        Initialize a message.

        :param drone_id: ID of this drone, generated at runtime.
        :param location: Current location, as a GPSData object.
        :param dBm: dBm value at this frequency.\
        :param sdr: raw IQ samples
        """
        self.triangle_count = triangle_count
        self.vertex = vertex
        self.timestamp = time.time()
        self.lat = location.lat
        self.lon = location.lon
        self.alt = location.alt
        self.heading = heading
        self.dBm = dBm

    def serialize(self):
        """
        Generate a string representation of this message for logging to disk.

        :return: JSON representation of the message.
        """
        return {

            'triangle_info': {
                'count': self.triangle_count,
                'v': self.vertex,
            },
            'timestamp': self.timestamp,
            'location': {
                'lat': self.lat,
                'lon': self.lon,
                'alt': self.alt,
            },
            'heading': self.heading,
            'dBm': self.dBm,
        }

    @staticmethod
    def deserialize(json):
        return HotspotWirelessMessage(
            location=GPSData(
                lat=json['location']['lat'],
                lon=json['location']['lon'],
                alt=json['location']['alt'],
            ),
            heading=json['heading'],
            dBm=json['dBm'],
            vertex=json['vertex'],
            triangle_count=json['triangle_count']
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

class ExchangedData(BaseMessage):
    """
    Definition of the structured message to periodically log sensed wireless data to disk.
    """

    def __init__(self, alpha, epsilon, shared_data):
        """
        Initialize a message.

        :param drone_id: ID of this drone, generated at runtime.
        :param location: Current location, as a GPSData object.
        :param dBm: dBm value at this frequency.\
        :param sdr: raw IQ samples
        """
        self.alpha = alpha
        self.epsilon = epsilon
        self.shared_data = shared_data

        # self.FLAG = FLAG

    def serialize(self):
        """
        Generate a string representation of this message for logging to disk.

        :return: JSON representation of the message.
        """
        return {
            'alpha': self.alpha,
            'epsilon': self.epsilon,
            'shared_data': self.shared_data,
        }

    @staticmethod
    def deserialize(json):
        return ExchangedData(
            alpha=json['alpha'],
            epsilon=json['epsilon'],
            shared_data=json['shared_data']
        )
