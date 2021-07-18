from enum import Enum


class DroneFormation:
    """
    DroneFormation is meant to represent a particular formation of multiple drones,
    such as a triangle or square formation.
    """

    def __init__(self, drone_count, radius):
        """
        Initialize a DroneFormation instance and start with a default formation based on the number of drones

        :param drone_count: the number of drones that will be in the formation
        :param radius: the length of the 'radius' that the formation should have. Ultimately this
        will be the distance from the formation center to each drone
        """
        self.drone_count = drone_count
        self.offsets = []

        # Assign default formation
        if drone_count == 2:
            self.form_line_horizontal(radius)
        elif drone_count == 3:
            self.form_triangle(radius)
        elif drone_count == 4:
            self.form_square(radius)

    def form_line_vertical(self, radius):
        """
        This is a 2-Drone formation
        Forms a vertical line formation: which should look like the following (top down view):

        X


        X

        :param radius: The distance from the center to each of the drones
        :returns: Whether the formation could successfully be formed given the number of drones in the formation
        """
        if self.drone_count != 2:
            return False

        self.form_from_angles(
            angles=[
                0,        # Top
                180,    # Bottom
            ],
            radius=radius,
        )

        return True

    def form_line_horizontal(self, radius):
        """
        This is a 2-Drone formation
        Forms a horizontal line formation: which should look like the following (top down view):

        X     X

        :param radius: The distance from the center to each of the drones
        :returns: Whether the formation could successfully be formed given the number of drones in the formation
        """
        if self.drone_count != 2:
            return False

        self.form_from_angles(
            angles=[
                90,        # Right
                270,    # Left
            ],
            radius=radius,
        )

        return True

    def form_triangle(self, radius):
        """
        This is a 3-Drone formation
        Forms a triangle formation: which should look like the following (top down view):
 
           X


        X     X

        :param radius: The distance from the center to each of the drones
        :returns: Whether the formation could successfully be formed given the number of drones in the formation
        """
        if self.drone_count != 3:
            return False

        self.form_from_angles(
            angles=[
                0,        # Top
                120    ,    # Right
                240,    # Left
            ],
            radius=radius,
        )

        return True

    def form_square(self, radius):
        """
        This is a 4-Drone formation
        Forms a square formation: which should look like the following (top down view):
 
        X     X


        X     X

        :param radius: The distance from the center to each of the drones
        :returns: Whether the formation could successfully be formed given the number of drones in the formation
        """
        if self.drone_count != 4:
            return False

        self.form_from_angles(
            angles=[
                45,        # Top right
                135,    # Bottom right
                225,    # Bottom left
                315,    # Top left
            ],
            radius=radius,
        )

        return True

    def form_from_angles(self, angles, radius):
        """
        Form a formation given a list of the angles that the drones should be at as well as the radius of the formation
        :param angles: a list of angles
        :param radius: the distance from the center to each of the drones
        """
        for angle in angles:
            self.offsets.append(FormationOffset(
                bearing = angle,
                distance = radius,
            ))

    def get_offset(self, drone_num):
        """
        Gets the offset of a single drone in the formation
        :param drone_num: the index of the drone in the formation (starting at 0)
        :returns: a FormationOffset instance representing that drone's offset in the formation
        """
        return self.offsets[drone_num]

    def rotate_by(self, angle):
        for i in range(len(self.offsets)):
            self.offsets[i].rotate_by(angle)


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

    def serialize(self):
        """
        Generate a string representation of this message for sending to other drones

        :returns: JSON representation of the message.
        """
        return {
        	'bearing': self.bearing,
        	'distance': self.distance
        }

    @staticmethod
    def deserialize(json):
        return FormationOffset(
            json['bearing'],
            json['distance']
        )

