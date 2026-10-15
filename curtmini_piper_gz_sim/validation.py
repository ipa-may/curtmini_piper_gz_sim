import math


def require_keys(value, expected, label):
    if not isinstance(value, dict):
        raise RuntimeError(f'{label} must be a YAML mapping')
    missing = set(expected) - set(value)
    extra = set(value) - set(expected)
    if missing or extra:
        raise RuntimeError(
            f'{label} has missing keys {sorted(missing)} and '
            f'unknown keys {sorted(extra)}'
        )


def finite_number(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RuntimeError(f'{label} must be a number')
    if not math.isfinite(value):
        raise RuntimeError(f'{label} must be finite')
    return float(value)
