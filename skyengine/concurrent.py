import time
from functools import wraps
from multiprocessing import Manager

from skyengine.exceptions import FlightAbortedException


class synchronized(object):
    """
    A decorator that causes a wrapped function to block until it has acquired all of a specified
    list of locks. Upon completion of the function, releases all locks.

    For example:
    ```
    @synchronized(lock1, lock2)
    def func(x):
        ...
    ```
    would prevent func from running until some_lock and some_other_lock were seized. Then, on
    return, it would release both of these locks.
    """
    def __init__(self, *args):
        # Induce a lock ordering to avoid deadlock caused by two threads attempting to acquire
        # the same locks in a different order.
        self._ordered_locks = list(sorted(args))

    def __call__(self, f):
        @wraps(f)
        def wrapped_f(*args, **kwargs):
            try:
                self._acquire_locks()
                return f(*args, **kwargs)
            finally:
                self._release_locks()
        return wrapped_f

    def _acquire_locks(self):
        """
        Seize all locks specified in the decorator.
        """
        [l.acquire() for l in self._ordered_locks]

    def _release_locks(self):
        """
        Release all locks specified in the decorator.
        """
        [l.release() for l in self._ordered_locks]


def sharing_attrs(*args):
    """
    This is a class decorator factory that automatically synchronizes attributes of
    a class between processes.

    For example:
    ```
    @sharing_attrs("x")
    class Foo(object):
        def __init__(self):
            self.x = 3

    instance = ()

    def print_x(instance):
        time.sleep(10)
        print instance.x

    Process(target=print_x, args=(instance,)).start()
    instance.x = 10
    print instance.x
    ```
    Would cause "10" to be printed twice, since the "x" attribute is synchronized
    between running processes, even though it was only changed in one process.

    Because this uses a multiprocessing.Manager() internally, the usual caveats for
    what can and cannot be Managed apply. No guarantees are made as to the atomicity of
    changes to the synchronized attributes.

    :param args: Names of attributes to share amongst processes.
    :return: A class decorator.
    """
    shared_attrs = frozenset(args)

    def decorator(cls):
        """
        Actual decorator with attribute names of interest 'baked in'.
        """
        def __init__(self, *args, **kwargs):
            # Create and start a managed namespace. This will intercept any attribute
            # lookups and redirect them to the managed namespace instead, where their
            # values will be shared amongst processes.
            self.__manager = Manager()
            # Note that while a Namespace() object seems appealing, it will silently
            # ignore any attributes that start with '_'.
            self.__managed_namespace = self.__manager.dict()
            self.__orig_init__(*args, **kwargs)

        def __getattribute__(self, attr):
            # Intercept specified attribute lookups.
            if attr in shared_attrs:
                return self.__managed_namespace.get(attr)
            # Use object's __getattribute__ to avoid infinite recursion
            return object.__getattribute__(self, attr)

        def __setattr__(self, attr, new_value):
            # Intercept specified attribute assignments.
            if attr in shared_attrs:
                self.__managed_namespace[attr] = new_value
            # Use object's __setattr__ to avoid infinite recursion
            object.__setattr__(self, attr, new_value)

        setattr(cls, "__orig_init__", cls.__init__)
        setattr(cls, "__init__", __init__)
        setattr(cls, "__getattribute__", __getattribute__)
        setattr(cls, "__setattr__", __setattr__)
        return cls

    return decorator


def blocking_poll(predicate, delay_sec, timeout_sec=None, abort_event=None):
    """
    Run predicate repeatedly, until it succeeds, delaying by delay_sec between
    retrying.

    :param predicate: A predicate to run
    :param delay_sec: A number of seconds to delay
    :param timeout_sec: A number of seconds after which to give up.
    :param abort_event: An Event which, when set, should cause the current operation
      to abort immediately within the next polling cycle and raise a
      FlightAbortedException.
    :return: Returns True if the predicate succeeds, False if the timeout occurred  first.
    """
    start_time = time.time()
    # Check once before even running the polling cycle - we'd like to be able
    # to abort a poll before the first cycle.
    if abort_event is not None and abort_event.is_set():
        raise FlightAbortedException()

    while not predicate():
        if abort_event is not None and abort_event.is_set():
            raise FlightAbortedException()
        current_time = time.time()
        if timeout_sec is not None:
            if (current_time - start_time) > timeout_sec:
                return False
        time.sleep(delay_sec)
    return True
