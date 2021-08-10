import signal

from flask import Flask
from flask import jsonify
from flask import request
from redis import StrictRedis

import skymission
from callback import CallbackContext
from logger import Logger

# Create a module-scoped callback store.
# Unfortunately, there does not exist a way to store the context on a per-mission basis while
# allowing access to the decorator as a class property in Mission subclasses.
ctx = CallbackContext()
callback = ctx.create_callback
panic = ctx.create_panic_callback


def noop(*args, **kwargs):
    pass


class Mission(object):
    """
    Generic mission superclass.
    """

    # Mission-specific HTTP server for externally pushed commands
    app = Flask(__name__)

    # String ID of this mission, for use by Skyserve when determining how to route mission commands.
    mission_id = None

    # Port for the mission-specific HTTP server
    port = 4000

    # Logger to standard output for mission debugging
    log = Logger()

    # Redis client for self-registering ports for local service discovery
    redis = StrictRedis()

    # Nullary SIGINT callback function
    # Skymission will make a best-effort attempt to run this logic before quitting the process.
    on_exit = noop

    @property
    def _mission_key(self):
        """
        Name of the Redis key pairing this mission with its associated service port.
        """
        return 'skymission:mission:{mission_id}'.format(mission_id=self.mission_id)

    def start_server(self):
        """
        After performing all other initialization procedures, start the HTTP server for external
        clients to push commands to the local drone. Note that this must be the LAST invocation in
        __init__, since `self` is captured when registering HTTP handlers as part of starting the
        server. (Otherwise, callbacks use a stale, captured `self` object when invoked.)
        """
        def signal_handler(signal, frame):
            self._deregister_service()
            self.on_exit()
            default_sigint_handler()

        # Stop the server when the mission application exits with an interrupt signal.
        default_sigint_handler = signal.getsignal(signal.SIGINT)
        signal.signal(signal.SIGINT, signal_handler)

        self._register_internal_callbacks()
        self._register_mission_callbacks()
        self._register_service()
        self.app.run(
            host='0.0.0.0',
            port=self.port,
            threaded=True,
        )

    def register_cleanup(self, on_exit):
        """
        Register a cleanup function to invoke when the process quits.

        :param on_exit: Nullary function to invoke when the mission receives a SIGINT.
        """
        self.on_exit = on_exit

    def enable_disk_event_logging(self):
        """
        Optionally enable recording mission event logs to disk as well as printing to standard
        output.
        """
        self.log.enable_disk_logging(self.mission_id)

    def _register_service(self):
        """
        Register the service ID-port binding.
        """
        if not self.mission_id:
            raise SkymissionException(
                'mission_id must be a non-null string uniquely identifying this mission.'
            )

        self.redis.set(self._mission_key, self.port)

    def _deregister_service(self):
        """
        Deregister the service ID-port binding.
        """
        self.redis.delete(self._mission_key)

    def _register_internal_callbacks(self):
        """
        Register all internal mission callbacks. Internal callbacks are always available for every
        mission, independent of those explicitly implemented by the client application.
        """
        factories = InternalHandlers(self).get_factories()

        for handler in map(apply, factories):
            self._register_handler(
                endpoint=handler['endpoint'],
                name=handler['name'],
                func=handler['func'],
            )

    def _register_mission_callbacks(self):
        """
        Register all declared callback functions within the current callback context. This adds the
        route to the local HTTP server and assigns the corresponding class method as a callback
        function, ensuring to preserve the state of `self` across requests.
        """
        def with_json(func):
            def handler(*args, **kwargs):
                input_data = request.get_json(force=True, silent=True)
                is_local = request.headers.get('X-Forwarded-For') in ['127.0.0.1', '::1']
                ret = func(self, input_data, is_local, *args, **kwargs)
                return jsonify(ret or {})

            return handler

        for cb in ctx.get_callbacks():
            self._register_handler(
                endpoint=cb.endpoint,
                name=cb.func.__name__,
                func=with_json(cb.func),
            )

    def _register_handler(self, endpoint, name, func):
        """
        Register an HTTP handler.

        :param endpoint: Path for the handler.
        :param name: Name of the handler. This must be unique across all handlers.
        :param func: The view function to invoke when the path is requested.
        """
        self.app.add_url_rule(
            rule=endpoint,
            endpoint=name,
            view_func=func,
            methods=['GET', 'POST'],
        )


class InternalHandlers(object):
    """
    Nullary factories for internal handlers available on every mission.
    """

    def __init__(self, mission_context):
        """
        Create a wrapper for internal handler factories.

        :param mission_context: Execution context (self) of the running mission.
        """
        self.mission_context = mission_context

    def get_factories(self):
        """
        Retrieve all internal handler factories.

        :return: A list of all nullary factories for internal handlers.
        """
        return [
            self.endpoints_factory,
            self.panic_factory,
            self.version_factory,
            self.event_log_factory,
        ]

    @staticmethod
    def endpoints_factory():
        """
        Endpoint for listing all public endpoints available for a mission.
        """
        def endpoints():
            return jsonify({
                'endpoints': [
                    {
                        'endpoint': cb.endpoint,
                        'description': cb.description,
                        'required_params': cb.required_params,
                    }
                    for cb in ctx.get_callbacks()
                    if cb.public
                ],
            })

        return dict(
            endpoint='/_/endpoints',
            name='_internal_endpoints',
            func=endpoints,
        )

    def panic_factory(self):
        """
        Endpoint for emergency panicking the drone.
        """
        panic_callback = ctx.panic_callback
        if not panic_callback:
            raise SkymissionException(
                'Every mission is required to register a panic handler using the @panic decorator!'
            )

        def panic_func():
            return jsonify(panic_callback(self.mission_context) or {})

        return dict(
            endpoint='/_/panic',
            name='_internal_panic',
            func=panic_func,
        )

    @staticmethod
    def version_factory():
        """
        Endpoint for querying the Skymission version in use by the mission.
        """
        def version():
            return jsonify({
                'version': skymission.__version__,
            })

        return dict(
            endpoint='/_/version',
            name='_internal_version',
            func=version,
        )

    def event_log_factory(self):
        """
        Endpoint for reading recent mission event log messages.
        """
        def event_log():
            data = request.get_json(force=True, silent=True) or {}
            return jsonify(self.mission_context.log.read_history(n=data.get('num_messages', 10)))

        return dict(
            endpoint='/_/event-log',
            name='_internal_event_log',
            func=event_log,
        )


class SkymissionException(Exception):
    """
    Exception raised when the client mission application configuration has some error.
    """
    pass
