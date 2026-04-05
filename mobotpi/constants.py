import math 

WHEEL_R = 0.0425 # m
#center of wheel to center of wheel
WHEELBASE =  0.245 # m
CHASSIS_SPEED = 1 # m/s
MAX_WHEEL_RPM = 6500 # rpm --> calculated based on motors going 650 RPM/Volt at 22 Volts split across 2 wheels

# just picked a lat long near the start of the course.
ORIGIN_LAT = 40.442169
ORIGIN_LON =  -79.945086

def distance(p1: tuple[float, float], p2: tuple[float, float]) -> float:
    """Euclidean distance between two 2D points."""
    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])
 
def normalize_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi
 
def latlon_to_xy(
    lat: float,
    lon: float,
) -> tuple[float, float]:
    """
    Flat-earth projection from lat/lon to meters relative to an origin.

    Returns (x, y) where x = east, y = north.
    """
    METERS_PER_DEG_LAT = 111_320
    meters_per_deg_lon = 111_320 * math.cos(math.radians(ORIGIN_LAT))

    x = (lon - ORIGIN_LAT) * meters_per_deg_lon
    y = (lat - ORIGIN_LON) * METERS_PER_DEG_LAT

    return (x, y)
