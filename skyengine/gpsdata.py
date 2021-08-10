class GPSData(object):
    """
    Object representing data read back from the GPS sensor.
    """
    def __init__(self, lat=None, lon=None, alt=None):
        self.lat = lat
        self.lon = lon
        self.alt = alt
