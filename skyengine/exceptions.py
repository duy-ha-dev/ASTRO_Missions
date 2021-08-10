class RicePublicRelationsException(Exception):
    """
    As we all know, Rice's behavior in the public sphere is usually governed by
    the equation
    $argmax_{a \in A} PublicRelations(a)$, where $A$ is the set of all possible
    administratively-sanctioned behaviors.

    Since a pesky undergrad could demand that the drone perform an action that might threaten Rice's
    (self-awarded?) title of "Harvard of the South" or act out of line with the principle(s?) that
    guide William Marsh Rice University, we've created an exception to catch these behaviors.
    For example, if a user attempted to fly a drone 50 miles to the West at 100 m/s, accidentally
     swap columns of the ELEC494 Cycle 0 product development worksheet, or suggest that the Rice
     Athletics department send less cringe-worthy emails, a RicePublicRelationsException should be
     raised posthaste. Doing otherwise just wouldn't be sound Business Wisdom.
    """
    pass


class AvionicsException(RicePublicRelationsException):
    """
    An exception for avionics-related troubles.
    """
    pass


class FlightAbortedException(RicePublicRelationsException):
    """
    An exception raised when a flight needs to be aborted.
    """
    pass
