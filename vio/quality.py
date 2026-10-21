"""Basic VIO validity checks; these cannot establish absolute accuracy."""
import math


def invalid_reason(values, receipt_age, stamp_age, tracking_ok, tracking_age):
    if not tracking_ok or tracking_age > 1.0:
        return "VINS is initializing, resetting, or its tracking status is stale"
    if receipt_age > 1.0 or not -0.05 <= stamp_age <= 0.5:
        return "VIO output is missing, delayed, or from an old simulation clock"
    if not all(math.isfinite(value) for value in values):
        return "VIO output contains nonfinite values"
    if abs(math.hypot(*values[3:7]) - 1.0) > 0.025:
        return "VIO orientation is not a unit quaternion"
    if math.hypot(*values[7:]) > 3.0:
        return "VIO speed exceeds the configured simulation validity limit"
    if math.hypot(*values[:3]) > 100.0:
        return "VIO position exceeds the configured simulation validity limit"
    return None
