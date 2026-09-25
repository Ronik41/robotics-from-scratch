"""Pure measurement models, separate from estimation and control."""
import math


def encoder_count(angle, origin, resolution):
    if not all(math.isfinite(v) for v in (angle, origin)) or resolution <= 0:
        raise ValueError('Invalid encoder sample/resolution')
    return round((angle-origin)*resolution/(2*math.pi))
