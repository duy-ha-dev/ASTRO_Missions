import json

import file_read_backwards


def read_log(path, deserializer=lambda val: val, reverse=False):
    """
    Read a log file on disk.

    :param path: Absolute or relative path to the on-disk persistent log file to read.
    :param deserializer: Unary function dictating how JSON-serialized log entries should be
                         transformed into an instance of a BaseMessage subclass.
    :param reverse: True to read the entries in reverse order (most recent entries first).
    :return: An iterable (generator) of log entries in the order they appear in the log file.
    """
    def transform(serialized):
        return deserializer(json.loads(serialized))

    if not reverse:
        with open(path, 'r') as log:
            for line in log:
                yield transform(line)
    else:
        with file_read_backwards.FileReadBackwards(path) as log:
            for line in log:
                yield transform(line)
