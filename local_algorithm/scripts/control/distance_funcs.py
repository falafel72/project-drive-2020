#!/usr/bin/env python
import numpy as np


def distance_score(distances, exp, threshold):
    # Translates distance into a cost.
    # Very high cost if distance under a set threshold,
    # and a polynomially decaying cost otherwise
    mask = np.less(distances, threshold)
    return np.power(distances, exp) * (1 - mask) + 1000 * mask


def length_weight(num_steps, exp):
    # This uses a polynomial decay to give more importance
    # to points closer to the car
    # Returns an array with sum of 1
    weights = np.zeros(num_steps)
    for k in range(num_steps):
        weights[k] = (k + 10) ** exp
    return weights / sum(weights)
