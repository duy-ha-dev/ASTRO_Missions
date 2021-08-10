from functools import wraps


class Callback(object):
    """
    Wraps properties of a Skymission callback.
    """

    def __init__(self, endpoint, description, func, required_params, public):
        self.endpoint = endpoint
        self.description = description
        self.func = func
        self.required_params = required_params
        self.public = public


class CallbackContext(object):
    """
    Store for registered mission callbacks.
    """

    def __init__(self):
        """
        Create a callback context.
        """
        self.callbacks = []
        self.panic_callback = None

    @property
    def create_panic_callback(self):
        """
        Create a callback for the mandatory panic handler.
        """
        def panic_decorator(func):
            self.panic_callback = func
            return func

        return panic_decorator

    def create_callback(
        self,
        endpoint,
        description=None,
        allow_local=True,
        required_params=(()),
        public=False,
    ):
        """
        Callback decorator factory. Used to create a callback decorator to use on Mission subclass
        methods. This method statefully adds a callback to the context when invoked.

        :param endpoint: Endpoint to register for the callback.
        :param description: String description of this callback's functionality.
        :param allow_local: True to allow the callback to be invoked by local requests.
        :param required_params: List of strings describing all required keys in the input data.
        :param public: True to indicate that this is a "public" endpoint that may be invoked by the
                       GCS (as opposed to only other drones running a mission)
        :return: A class method decorator for registering an endpoint to a function.
        """
        def callback_decorator(func):
            @wraps(func)
            def wrapped_func(*args, **kwargs):
                _, data, is_local = args

                if len(required_params) and not all(key in (data or {}) for key in required_params):
                    return {
                        'error': 'Required params are missing: {params}'.format(
                            params=list(set(required_params).difference(set(data or {}))),
                        )
                    }

                if is_local and not allow_local:
                    return

                return func(*args, **kwargs)

            cb = Callback(endpoint, description, wrapped_func, required_params, public)
            self.callbacks.append(cb)

            return wrapped_func

        return callback_decorator

    def get_callbacks(self):
        """
        Get a list of all stored callbacks in this context.

        :return: A list of callbacks formatted as a tuple of (endpoint, func).
        """
        return self.callbacks
