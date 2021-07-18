from geo import Coordinate
from geo import Region

# Meters apart of each drone's flight altitude. 
# This helps avoid in-flight collisions
VERTICAL_SEPARATION = 2.0	

# Seconds apart to launch each drone.
# This allows us to have time to make sure each drone is flying 
# properly and react properly if there is an issue at takeoff
STAGGER_LAUNCH_SEC = 5.0

# Meters apart that the drones must minimally be placed apart
MIN_LATERAL_SEPARATION = 5.0

# Meters that the drone cluster can maximally start away from the chosen region
# This check helps avoid major GPS error issues or simply starting the drones in the wrong location
MAX_DISTANCE_FROM_REGION = 150.0

# Pre-defined regions of places where we might test in the real world
REGIONS = {
    'soccer': Region(
        nw=Coordinate(29.717682, -95.406094),
        ne=Coordinate(29.718003, -95.405334),
        sw=Coordinate(29.717247, -95.405860),
        se=Coordinate(29.717571, -95.405093),
    ),
    'rugby': Region(
        nw=Coordinate(29.718509, -95.406502),
        ne=Coordinate(29.718831, -95.405705),
        sw=Coordinate(29.718118, -95.406276),
        se=Coordinate(29.718452, -95.405433),
    ),
    'stadium': Region(
        nw=Coordinate(29.715956, -95.409520),
        ne=Coordinate(29.716697, -95.409538),
        sw=Coordinate(29.715959, -95.409110),
        se=Coordinate(29.716700, -95.409123),
    ),
    'eng-quad': Region(
        nw=Coordinate(29.720547, -95.399960),
        ne=Coordinate(29.720688, -95.399566),
        sw=Coordinate(29.720336, -95.399896),
        se=Coordinate(29.720499, -95.399498),
    ),
    'founders': Region(
        nw=Coordinate(29.719332, -95.397456),
        ne=Coordinate(29.719517, -95.396971),
        sw=Coordinate(29.719072, -95.397326),
        se=Coordinate(29.719260, -95.396843),
    ),
    'wiess-1': Region(
        nw=Coordinate(29.715703, -95.402911),
        ne=Coordinate(29.715990, -95.402230),
        sw=Coordinate(29.715324, -95.402722),
        se=Coordinate(29.715613, -95.402042),
    ),
    'wiess-2': Region(
        nw=Coordinate(29.714938, -95.402594),
        ne=Coordinate(29.715271, -95.401833),
        sw=Coordinate(29.714569, -95.402390),
        se=Coordinate(29.714899, -95.401629),
    ),
    'north-lot': Region(
        nw=Coordinate(29.719516, -95.405606),
        ne=Coordinate(29.719703, -95.405081),
        sw=Coordinate(29.719082, -95.405375),
        se=Coordinate(29.719292, -95.404863),
    ),
    'rec': Region(
        nw=Coordinate(29.719146, -95.404569),
        ne=Coordinate(29.719406, -95.403875),
        sw=Coordinate(29.718848, -95.404411),
        se=Coordinate(29.719106, -95.403717),
    ),
}

