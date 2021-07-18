import binascii
import os


def uuid():
    """
    Generate a UUID.

    :return: UUID consisting only of alphanumeric characters.
    """
    return binascii.b2a_hex(os.urandom(4))

