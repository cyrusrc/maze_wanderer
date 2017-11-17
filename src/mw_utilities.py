import math


def first_non_nan(ranges, forward):
    # TODO: first_non_nan
    """

    :param ranges:
    :param forward:
    :return:
    """
    if not forward:
        ranges = reversed(ranges)
    for e in ranges:
        if not math.isnan(e):
            return e
    return -1


def is_perpendicular(left, right):
    # TODO: is_perpendicular
    """

    :param left:
    :param right:
    :return:
    """
    return abs(left - right) < 0.01
