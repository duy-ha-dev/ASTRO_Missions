import threading
import time
from functools import wraps
from multiprocessing import Process
from multiprocessing import Queue


def fork_wait(func):
    """
    Fork a separate process in which to run an arbitrary function. Block until execution is complete
    and pass the return value back to the parent process.

    :param func: Function to execute.
    :return: Wrapped function that passes return value of the input function, after application.
    """
    @wraps(func)
    def wrapped_func(*args, **kwargs):
        store = Queue()

        def sync_run():
            ret = func(*args, **kwargs)
            store.put(ret)

        process = Process(target=sync_run)
        process.start()
        process.join()

        return store.get(timeout=1) if not store.empty() else None

    return wrapped_func


def async_process(func):
    """
    Fork a separate process and run an arbitrary function asynchronously.

    :param func: Function to execute.
    :return: Wrapped function that asynchronously executes the target function.
    """
    @wraps(func)
    def wrapped_func(*args, **kwargs):
        process = Process(target=func, args=args, kwargs=kwargs)
        process.start()
        return process
    return wrapped_func


def tick(interval=0):
    """
    Decorator factory for asynchronously and continuously executing a function periodically.

    :param interval: Number of seconds to delay between each tick.
    :return: A decorator that can be applied to a function to cause it to self-invoke periodically
             following the first invocation.
    """
    def decorator(func):
        @wraps(func)
        def wrapped_func(*args, **kwargs):
            terminate_flag = Queue()

            def async_run():
                while terminate_flag.empty():
                    func(*args, **kwargs)
                    time.sleep(interval)

            def cancel():
                terminate_flag.put(True)

            thread = threading.Thread(target=async_run, args=())
            thread.daemon = True
            thread.start()

            return cancel

        return wrapped_func

    return decorator
