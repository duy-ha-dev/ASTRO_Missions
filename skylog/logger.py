import json
from Queue import Empty
from Queue import Queue
from collections import deque

from concurrency import async
from message import BaseMessage


class Logger(object):
    """
    Abstraction for logging messages to disk with throttled disk I/O by buffering messages
    in-memory.
    """

    def __init__(self, path=None, in_memory=False, num_cached_messages=100, autostart=True):
        """
        Initialize a new logger.

        :param path: Absolute or relative path to the on-disk persistent log file. This parameter
                     must be specified if in_memory is False.
        :param in_memory: True to only retain log messages in memory. This disables writing any
                          messages to disk.
        :param num_cached_messages: Number of recent messages to persist in-memory to be available
                                    for immediate serving to clients that wish to read recently
                                    logged messages.
        :param autostart: True to automatically start the consumer thread on initialization. If
                          False, the client is expected to manually invoke start().
        """
        if not in_memory and not path:
            raise AttributeError('Log file path must be specified when logging messages to disk!')

        # Queue of messages pending writing to disk.
        self.disk_queue = Queue()
        # Cache of recently logged messages. New entries are appended from the left.
        self.message_cache = deque(maxlen=num_cached_messages)

        if not in_memory:
            # Boolean flag used to synchronize the background consumer thread. This flag's value is
            # mutated to be True to indicate that the consumer should finish any pending work and
            # terminate.
            self.should_terminate = False
            self.disk_file = open(path, 'a')

            if autostart:
                self.start()

    def start(self):
        """
        Start the consumer thread for writing log entries to disk.
        """
        self.consumer_thread = self._consume_messages()

    def log(self, message=None):
        """
        Log a message. Note that this method will *not* instantly write the message to disk, but
        will buffer it in-memory and queue it for writing to disk at some future point in time.

        :param message: Instance of a BaseMessage subclass describing the message to log.
        """
        if not message:
            return

        if not isinstance(message, BaseMessage):
            raise ValueError('Logged message must be a subclass of BaseMessage!')

        self.disk_queue.put(message)
        self.message_cache.appendleft(message)

    def read(self, n=10):
        """
        Read the n most recently logged messages.

        :param n: The number of recent messages to read.
        :return: A list of recently logged messages, in descending order of insertion time (most
                 recent messages first).
        """
        return list(self.message_cache)[:n]

    def flush(self):
        """
        Forcefully flush all buffered messages to disk.
        """
        self.disk_file.flush()

    @async
    def _consume_messages(self):
        """
        Asynchronous background task to continuously dump buffered log messages to disk, with one
        serialized message per log line.
        """
        # Loop forever, unless requested to terminate. If requested to terminate, continue
        # processing the queue until all pending messages are processed.
        while not self.should_terminate or not self.disk_queue.empty():
            try:
                # Try getting a message from the queue and writing to disk. Enforce a maximum
                # timeout in order to prevent the thread from indefinitely being stuck here.
                message = self.disk_queue.get(timeout=1)
                serialized = json.dumps(message.serialize())
                self._write_line(serialized)
            except Empty:
                continue

    def terminate(self):
        """
        Flush all remaining buffered messages to disk, terminate the consumer thread, and close the
        file descriptor. This function is stateful; the logger must be re-initialized to log any
        more messages.
        """
        # Set the flag to indicate to the consumer thread that it should finish processing all
        # pending work and terminate
        self.should_terminate = True
        # Block until the consumer thread to terminates
        self.consumer_thread.join()

        # Write all pending messages to disk immediately
        self.disk_file.flush()
        self.disk_file.close()

    def _write_line(self, line):
        """
        Write a single line to the open file descriptor.

        :param line: String to write to the file as a single line.
        """
        self.disk_file.write(line)
        self.disk_file.write('\n')


class DirectLogger(Logger):
    """
    Abstraction for logging messages to disk without any buffered or throttled disk I/O.
    """

    def __init__(self, path=None, in_memory=False, num_cached_messages=100):
        """
        Initialize a new logger that writes log messages to disk immediately. A DirectLogger, in
        contrast with a Logger, should be used in scenarios where log throughput is low and logging
        is more critical. Messages are flushed to disk at the earliest opportunity after every
        successful write.

        :param path: Absolute or relative path to the on-disk persistent log file. This parameter
                     must be specified if in_memory is False.
        :param in_memory: True to only retain log messages in memory. This disables writing any
                          messages to disk.
        :param num_cached_messages: Number of recent messages to persist in-memory to be available
                                    for immediate serving to clients that wish to read recently
                                    logged messages.
        """
        super(DirectLogger, self).__init__(
            path=path,
            in_memory=in_memory,
            num_cached_messages=num_cached_messages,
            autostart=False,
        )

    def start(self):
        """
        The direct logger is entirely synchronous and does not use any asynchronous background
        threads. This operation is a noop.
        """
        pass

    def log(self, message=None):
        """
        Log a message. This method will instantly write the message to disk.

        :param message: Instance of a BaseMessage subclass describing the message to log.
        """
        super(DirectLogger, self).log(message=message)

        message = self.disk_queue.get()
        serialized = json.dumps(message.serialize())
        self._write_line(serialized)
        self.flush()

    def terminate(self):
        """
        Terminating the logger simply closes the open file descriptor for writing messages.
        It is technically not strictly necessary to ensure that this is called before program
        termination.
        """
        self.disk_file.close()
