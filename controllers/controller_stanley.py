import math
import numpy as np


def calculate_steering_angle(v, e, phi, Ke, max_steering_angle):
    """
    Calculate the steering angle theta(t) based on the given equation.

    Parameters:
    v: Velocity at time t.
    e: Crosstrack error at time t.
    phi: Heading error at time t.
    Ke: Gain for the crosstrack error.

    Returns:
    psi: Calculated steering angle theta(t).
    """
    psi = phi + math.atan((Ke * e) / v)
    psi = np.clip(psi, -max_steering_angle, max_steering_angle)

    return psi
