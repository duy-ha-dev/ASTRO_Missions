"""
This module defines two classes, FlightController and DroneController,
which are responsible for encapsulating stateful flight controller behaviors
and drone flight behaviors.

DroneController inherits from FlightController - if really fine-grained operations
on the FC are required for a very "unusual" mission, then a FlightController can be
directly instantiated in place of a DroneController. However, for any missions we've
described, a DroneController is probably preferable.
"""
from functools import wraps
from multiprocessing import Event, RLock

import dronekit_sitl
from dronekit import LocationGlobal, VehicleMode, connect

from skyengine.concurrent import blocking_poll, sharing_attrs, synchronized
from skyengine.exceptions import AvionicsException, RicePublicRelationsException
from skyengine.flightstatus import FlightStatus
from skyengine.geometry import airborne_haversine_distance, offset2latlon
from skyengine.gpsdata import GPSData
from skyengine.singleton import Singleton

# 1/2 mile -- if a drone wanted to suddenly travel farther than this, something's amiss.
_MINIMUM_UNREASONABLE_DISTANCE_METERS = 8000000000

# Anything below 3 meters is potentially dangerous.
_MAXIMUM_UNREASONABLE_ALTITUDE = 3.0


@sharing_attrs("_status")
class FlightController(object):
    """
    Singleton class defining useful abstractions for interacting with the flight controller.
    """

    __metaclass__ = Singleton
    _locks = {
        "arm": RLock(),
        "connect": RLock(),
        "status": RLock(),
        }

    def __init__(self, fc_address=None, arm_timeout_sec=60, loc_start=None):
        """
        Connect to the flight controller and create a new Drone.

        :param fc_address: Address of the flight controller.
        :param arm_timeout_sec: Maximum time it should take to arm the drone
          before we give up.
        :param loc_start: A (lat,lon) pair used to initialize the GPS location of the drone when
          using SITL.
        """
        self._status = FlightStatus.NOT_READY
        self.fc_address = fc_address
        self._arm_timeout = arm_timeout_sec
        self._sitl = None
        self._vehicle = None
        self._loc_start = loc_start if not fc_address else None

        # Connect to the flight controller.
        self.connect()
        # The drone should be on the ground.
        self.status = FlightStatus.LANDED

    @property
    def status(self):
        return self._status

    @status.setter
    def status(self, new_status):
        """
        Once the status is PANICKED, prohibit setting other statuses.
        """
        if self._status is not FlightStatus.PANICKED:
            self._status = new_status

    @property
    def vehicle(self):
        """
        Getter for the 'vehicle', DroneKit's interface to the physical drone.
        If no connection is established, then this function guarantees that one
        will be attempted.
        """
        if self._vehicle is None:
            self.connect()
        return self._vehicle

    @synchronized(_locks["arm"])
    def ensure_armed(self):
        """
        Attempt to arm the vehicle if it is not already.
        Do NOT perform this step until a flight mode has been selected.

        :param timeout: The time after which we give up on arming the drone.
        """
        # If the vehicle is already armed, we're done.
        if self.vehicle.armed is True:
            return

        # Make sure the drone is ready to be armed.
        armable = blocking_poll(lambda: self.vehicle.is_armable, 0.5, self._arm_timeout)
        if not armable:
            raise AvionicsException("Drone is not armable after {}".format(self._arm_timeout))

        self._vehicle.armed = True
        # Arm the drone and wait for it to acknowledge being armed.
        arm_success = blocking_poll(lambda: self._vehicle.armed,
                                    0.25,
                                    timeout_sec=self._arm_timeout)
        if not arm_success:
            raise AvionicsException(
                "Asked drone to arm, but it refused. "
                "Could not arm within {}".format(self._arm_timeout)
                )

    @synchronized(_locks["connect"], _locks["status"])
    def connect(self):
        """
        Establish a connection with the vehicle's flight controller.
        If no connection string was specified, assume we're using SITL.

        This function also "downloads commands" from the flight controller.
        """
        # If a simultaneous connect attempt follows a successful one, ignore it.
        if self._vehicle is not None:
            return

        # Set up SITL if we're using it.
        if self.fc_address is None:
            if self._loc_start:
                lat, lon = self._loc_start
                self._sitl = dronekit_sitl.start_default(lat=lat, lon=lon)
            else:
                self._sitl = dronekit_sitl.start_default()
            self.fc_address = self._sitl.connection_string()

        # Connect to the flight controller.
        self._vehicle = connect(self.fc_address, wait_ready=True)

        # "Download commands" from the flight controller.
        # The fact that this is separate from the dronekit connect() function is puzzling.
        # Many common tasks (eg, reading the starting altitude of the drone) will fail until
        # we do so.

        # Repeatedly download commands until the vehicle acquires a home location.
        blocking_poll(
            self.poll_home_location,
            2,
            10
        )

    def read_gps(self):
        """
        Fetch the current location.
        :return: a GPSData object describing the current coordinates reported by
        DroneKit.
        """
        location = self.vehicle.location.global_frame
        return GPSData(location.lat, location.lon, location.alt)

    def poll_home_location(self):
        """
        Downloads the vehicle commands and returns the home location if it has been set.

        :return: The home_location of the vehicle. May be None.
        """
        cmds = self._vehicle.commands
        cmds.download()
        cmds.wait_ready()
        return self._vehicle.home_location

    def cleanup(self):
        """
        Stop SITL (if started) and close the connection to the vehicle (if opened).
        """
        if self._sitl is not None:
            self.sitl.stop()
        if self._vehicle is not None:
            self._vehicle.close()


# XXX: Ideally, this would be a staticmethod of the Drone class.
#      Unfortunately, trying to use a staticmethod on a class inside that class
#      as a decorator results in a TypeError in Python 2. So, it's a standalone
#      function now.
def _only_when(*args):
    """
    A decorator which will cause certain flight operations to be ignored
    unless the vehicle is in a particular mode. For example,
    ```
    @_only_when(FlightStatus.FOO, FlightStatus.BAR)
    def some_flight_method(self):
        ...
    ```
    would guarantee that Drone.some_flight_method() will be ignored unless
    the drone has status FlightStatus.FOO or FlightStatus.BAR.

    This function can be used to prevent invalid transitions on the Drone
    state machine.

    :param args: A list of FlightStatuses in which the command makes sense
    te perform.
    """
    permissible_statuses = frozenset(args)

    # Actual decorator created by _only_when.
    def _decorator(func):
        @wraps(func)
        def _wrapped_instance_method(*args, **kwargs):
            # Needs a "self" to check status
            if len(args) is 0:
                return
            if not isinstance(args[0], DroneController):
                return
            # Status must be one of the listed statuses.
            if args[0].status not in permissible_statuses:
                return
            return func(*args, **kwargs)
        return _wrapped_instance_method
    return _decorator


class DroneController(FlightController):
    """
    High-level drone-control abstractions, building on top of those for just the
    flight controller.
    """
    __metaclass__ = Singleton

    def __init__(self, *args, **kwargs):
        self.panic_event = Event()
        super(DroneController, self).__init__(*args, **kwargs)

    @synchronized(FlightController._locks["status"])
    @_only_when(FlightStatus.LANDED)
    def take_off(self, height_meters):
        """
        Ascend to a given height, in meters. Blocks until complete.

        :param height_meters: Height, in meters, to take off to.
        """
        self.vehicle.mode = VehicleMode("GUIDED")
        self.status = FlightStatus.TAKING_OFF
        self.ensure_armed()
        self.vehicle.simple_takeoff(height_meters)
        # Wait until we've reached the correct height.
        blocking_poll(
            lambda: self.vehicle.location.global_relative_frame.alt >= 0.95 * height_meters,
            0.25,
            abort_event=self.panic_event)
        self.status = FlightStatus.FLYING

    # FIXME: Once we have reliable low-altitude relative altitude detection,
    #        remove drop_meters and trust the drone to land itself.
    @synchronized(FlightController._locks["status"])
    @_only_when(FlightStatus.FLYING, FlightStatus.PANICKED)
    def land(self, drop_meters=0.5):
        """
        Land the drone. Blocks until complete.

        :param drop_meters: Height at which the drone should consider itself
          "landed" and drop. Because height readings tend to be higher than they
          should be at low altitude, this should help the drone not crash itself
          until we get the SONAR sensor operational.
        """
        self.vehicle.mode = VehicleMode("LAND")
        self.status = FlightStatus.LANDING
        # Drop the drone from a given height if requested.
        if drop_meters is not None:
            blocking_poll(
                lambda: self.vehicle.location.global_relative_frame.alt <= drop_meters,
                0.25,
                timeout_sec=120
                )
            self.vehicle.armed = False
        # Make sure the drone is *actually* landed before we declare that it's
        # actually landed.
        else:
            blocking_poll(
                lambda: self.vehicle.location.global_relative_frame.alt <= 0.1,
                0.25,
                timeout_sec=120
                )
        self.status = FlightStatus.LANDED

    def distance_to(self, abs_position):
        """
        For a (lat, lon, alt) triple, find an approximate distance between that point and the
        drone's current location.

        :param abs_position: The (lat, lon, alt) triple to find the distance to. This is an
          absolute position, with altitude measured from sea level.
        :return: the distance to that abs_position
        """
        frame = self.vehicle.location.global_frame
        return airborne_haversine_distance((frame.lat, frame.lon, frame.alt), abs_position)

    @synchronized(FlightController._locks["status"])
    @_only_when(FlightStatus.FLYING)
    def goto(self, coords, altitude, airspeed=1, radius=2.5, timeout_sec=None):
        """
        Blocking function that tells the drone to go somewhere.
        The drone must be flying and asked to go somewhere sane.

        :param coords: A (lat, lon) double; both specified in an absolute reference frame.
        :param altitude: An altitude specified relative to the height the drone acquired its GPS
          fix at.
        :param airspeed: The speed, in m/s, that the drone should travel through these
          air.
        :param radius: The maximum distance (in meters) the drone is permitted to be
          from the position specified before considering itself arrived.
        :param timeout_sec: The length of time the drone has to get there.
        """
        # Make sure we're off to someplace "sane."
        lat, lon = coords
        home_altitude = self.vehicle.home_location.alt
        position = (lat, lon, altitude + home_altitude)
        self._validate_position(position)

        # Tell the drone to go there.
        self.vehicle.simple_goto(LocationGlobal(*position), airspeed=airspeed)

        # Wait for the drone to arrive.
        arrived = blocking_poll(
            lambda: self.distance_to(position) <= radius,
            0.25,
            timeout_sec=timeout_sec,
            abort_event=self.panic_event
            )
        if not arrived:
            raise AvionicsException("Drone could not make it to {}.".format(position))

    @synchronized(FlightController._locks["status"])
    @_only_when(FlightStatus.FLYING)
    def goto_relative(self, coords, altitude, **kwargs):
        """
        Blocking function that tells the drone to go somewhere, relative to its
        home position (ie, where it first acquired the GPS lock).
        All keyword arguments are identical to the goto() function.

        :param coords: A (delta_lat, delta_lon) double, both specified relative
          to the drone's starting location.
        :param altitude: An altitude relative to the drone's starting location to
          go to.
        """
        delta_lat, delta_lon = coords
        home = self.vehicle.home_location
        self.goto((home.lat + delta_lat, home.lon + delta_lon), altitude, **kwargs)

    @synchronized(FlightController._locks["status"])
    @_only_when(FlightStatus.FLYING)
    def move_by(self, offset, altitude, **kwargs):
        """
        Blocking function that moves the drone from its current position by
        an offset specified in meters, and to an altitude specified relative to
        the drone's home location.

        :param offset: A (meters_north, meters_east) pair specifying where the
          drone should move relative to its current location.
        :param altitude: The desired altitude for the drone, relative to the
          elevation at the drone's home location.
        """
        m_north, m_east = offset
        here = self.read_gps()
        d_lat, d_lon = offset2latlon(m_north, m_east, here.lat)
        self.goto((here.lat + d_lat, here.lon + d_lon), altitude, **kwargs)

    @synchronized(FlightController._locks["status"])
    @_only_when(FlightStatus.FLYING)
    def elevate(self, altitude, **kwargs):
        """
        Blocking funciton that moves the drone to an altitude, specified relative
        to the drone's home location. The drone should move only vertically.

        :param altitude: An altitude, in meters, that the drone should fly to.
        :param **kwargs: Keyword arguments forwarded to `DroneController.goto`.
        """
        self.move_by((0.0, 0.0), altitude, **kwargs)

    def panic(self):
        """
        Panic a drone. This aborts the flight immediately, causes the drone to
        land, and makes it unresponsive to any further movement-related commands
        issued through the DroneController.

        This should be used only in circumstances where Something Very Bad (TM)
        occurs, as requires that the drone be stopped immediately.
        """
        # Trigger the panic event. This should abort all polling events.
        self.panic_event.set()
        # Mark the drone as panicked.
        self.status = FlightStatus.PANICKED
        # Land the drone.
        self.land()

    def _validate_position(self, position):
        """
        Guarantee that moving the drone to a new position is "sane." Raise a
        RicePublicRelationsException if the position provided is suspect.
        :param position: a (lat, lon, altitude) 3-tuple containing the target position
        of the drone.
        """

        # Having the drone run away is not a university-sanctioned recreational activity.
        if self.distance_to(position) >= _MINIMUM_UNREASONABLE_DISTANCE_METERS:
            raise RicePublicRelationsException(
                "Drone tried to run away to {} (presumably the location of a Peer "
                "Institution).".format(position)
                )

        # Make sure our flying machine isn't trying to become a digging machine.
        _, _, requested_alt = position
        if (requested_alt - self.vehicle.home_location.alt) < _MAXIMUM_UNREASONABLE_ALTITUDE:
            raise RicePublicRelationsException(
                "Flying a drone underground is a poor use of university resources."
                )
