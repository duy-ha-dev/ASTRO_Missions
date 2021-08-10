import itertools
import time
from collections import deque


class LogLevel:
    """
    Mapping of log level enums to names.
    """
    DEBUG = 'DEBUG'
    INFO = 'INFO'
    WARN = 'WARN'
    ERROR = 'ERROR'


class Logger:
    # Optional event logging to disk
    disk_file = None

    # In-memory cache of mission event logs for future fast retrieval
    message_cache = deque(maxlen=500)

    def enable_disk_logging(self, mission_id):
        """
        Enable logging all messages to disk, one message per line. This function is idempotent.

        :param mission_id: ID of the currently active mission.
        """
        if self.disk_file:
            return

        date = time.strftime('%m-%d-%Y')
        hms = time.strftime('%H-%M-%S')
        log_file = '{prefix}_{mission_id}_{date}_{hms}'.format(
            prefix='mission-log',
            mission_id=mission_id,
            date=date,
            hms=hms,
        )
        self.disk_file = open(log_file, 'a')

    def read_history(self, n=10):
        """
        Read the most recently logged event messages.

        :param n: Number of events to retrieve.
        :return: A list of recently logged event messages, in descending order of insertion time
                 (most recent messages first).
        """
        return list(itertools.islice(self.message_cache, 0, n))

    def debug(self, message):
        """
        Log a debug message.

        :param message: Message to log.
        """
        return self._print_log(LogLevel.DEBUG, message)

    def info(self, message):
        """
        Log an info message.

        :param message: Message to log.
        """
        return self._print_log(LogLevel.INFO, message)

    def warn(self, message):
        """
        Log a warning message.

        :param message: Message to log.
        """
        return self._print_log(LogLevel.WARN, message)

    def error(self, message):
        """
        Log an error message.

        :param message: Message to log.
        """
        return self._print_log(LogLevel.ERROR, message)

    def _print_log(self, level, message):
        """
        Print a log entry to standard output, with the timestamp, log level, and context name
        automatically prefixed.

        :param level: Target log level.
        :param message: Message to log.
        """
        hms = time.strftime('%H:%M:%S')
        self._print_stdout(
            '[{hms}] [{level}] {message}'.format(
                hms=hms,
                level=level,
                message=message,
            )
        )

    def _print_stdout(self, line):
        """
        Print a line to standard output and optionally write it to the mission event log.

        :param line: Line to print and write to the mission event log.
        """
        print line
        self.message_cache.appendleft(line)

        if self.disk_file:
            self.disk_file.write(line)
            self.disk_file.write('\n')
            self.disk_file.flush()
