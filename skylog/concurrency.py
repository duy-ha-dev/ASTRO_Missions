import threading
from functools import wraps


def async(func):
    """
    Function decorator to make a function run asynchronously, by invoking it from a separate thread.

      @async
      def async_function():
          ...

    :return: A decorator function that can be used to decorate arbitrary functions.
    """
    @wraps(func)
    def async_func(*args, **kwargs):
        def task():
            func(*args, **kwargs)

        # Execute the original function in a separate thread, returning immediately
        thread = threading.Thread(target=task, args=())
        thread.daemon = True
        thread.start()

        return thread

    return async_func
