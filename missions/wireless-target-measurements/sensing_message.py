import time

from skyengine.gpsdata import GPSData
from skylog.message import BaseMessage


class SensingMessage(BaseMessage):
    """
    Definition of the structured message to periodically log sensed wireless data to disk.
    """

    def __init__(self, location, heading, dBm):
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
        self.heading = heading
        self.dBm = dBm

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
            'heading': self.heading,
            'wireless': {
                'dBm': self.dBm,
            },
        }

    @staticmethod
    def deserialize(json):
        return SensingMessage(
            location=GPSData(
                lat=json['location']['lat'],
                lon=json['location']['lon'],
                alt=json['location']['alt'],
            ),
            heading=json['heading'],
            dBm=json['wireless']['dBm'],
        )
