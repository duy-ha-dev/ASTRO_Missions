from geo import Coordinate
from geo import Region

# Whether the summoner should wait for the others to arrive before calculating a new hotspot
SHOULD_WAIT = True

IS_ALPHA_CONTINUOUS = True

HARDCODE_ALPHA = False

SIMULATED_ALPHA = -20.0
SIMULATED_EPSILON = 0.5

# SIMULATED_ALPHA = -10.465
# SIMULATED_EPSILON = 4.172

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
        nw=Coordinate(29.715645, -95.403260),
        ne=Coordinate(29.716095, -95.402107),
        sw=Coordinate(29.715196, -95.402989),
        se=Coordinate(29.715622, -95.401874),
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
