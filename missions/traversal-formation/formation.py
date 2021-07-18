class FormationOffset:
    """
    The bearing and distance offset of a particular drone from the center of the formation
    """

    def __init__(self, bearing, distance):
        """
        Initializes a FormationOffset instance

        :param bearing: the bearing pointing FROM the formation center TO the target drone (in degrees)
        :param distance: the distance from the formation center to the target drone (in meters)
        """
        self.bearing = bearing
        self.distance = distance

    def __repr__(self):
        """
        Generate string representation of the formation offset instance
        :returns: the string representation of the formation offset instance
        """
        return "Bearing: {bearing}, Distance: {distance}".format(
                bearing = self.bearing,
                distance = self.distance,
            )

    def rotate_by(self, angle):
        """
        Rotate this formation by the given amount of degrees, in the clockwise direction
        This method modifies the self.bearing property

        :param angle: the angle to rotate the offset by, in degrees. Rotation will be clockwise
        """
        self.bearing += angle

        # Ensure that the bearing remains within [0, 360]
        while self.bearing < 0:
            self.bearing = self.bearing + 360

        while self.bearing > 360:
            self.bearing = self.bearing  - 360

    def offset_coordinate(self, coordinate):
        """
        Offset a coordinate based on the current offset from the formation center

        :param coordinate: the coordinate of the target formation center
        :returns: the coordinate of the offset location
        """
        return coordinate.offset_bearing(self.bearing, self.distance)
