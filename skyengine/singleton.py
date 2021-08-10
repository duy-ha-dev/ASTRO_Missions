class Singleton(type):
    """
    Metaclass for singleton classes. See:
    https://stackoverflow.com/questions/6760685/creating-a-singleton-in-python#6798042
    """
    # The 'trick' here is that all classes declaring Singleton as their metaclass
    # will have an instance registered with Singleton when they're first instantiated.
    # Subseqent attempts to create new instances are intercepted and return the one
    # instantiated class.
    _instances = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]

    def _really_unsafe_reset_singletons(cls):
        """
        For use in testing, when you'd like a singleton class to stop acting like
        one.
        It is almost certainly a bad idea to use this outside of testing.
        """
        cls._instances = {}
