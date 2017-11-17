import math


def first_non_nan(ranges, forward):
    """Returns the first element of a list not of type NaN. Forward begins with list[0], not forward with list[-1].

    :param ranges: A list of ranges where each element represents the intensity of a scan reading.
    :param forward:
    :return:
    """
    if not forward:
        ranges = reversed(ranges)
    for e in ranges:
        if not math.isnan(e):
            return e
    return -1


def is_perpendicular(left, right, epsilon):
    """Compares leftmost and rightmost readings from a laserscan output. If their difference is less than the passed
       epsilon, that is considered to be perpendicular.

    :param left: A laserscan reading intensity
    :param right: A laserscan reading intensity
    :return:
    """
    return abs(left - right) < epsilon
