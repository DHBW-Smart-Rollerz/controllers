import numpy as np
from scipy.optimize import minimize
from scipy.linalg import solve_continuous_are


class LQRController:
    def __init__(self, Q=None, R=None):
        if Q is None:
            Q = np.diag([10, 10])
        if R is None:
            R = np.array([[1]])

        self.Q = Q
        self.R = R
        self.K_lqr = None

    def compute_gain(self, v, car_length=0.5):
        # Systemmatrizen definieren
        A = np.array([[0, 0], [0, v]])

        B = np.array([[v / car_length], [v]])

        # Lösen der Algebraischen Riccati-Gleichung
        P = solve_continuous_are(A, B, self.Q, self.R)

        # Berechne LQR-Gewinnmatrix K
        self.K_lqr = np.linalg.inv(self.R) @ B.T @ P

    def get_control_signal(self, v, angle_delta, offset):
        if self.K_lqr is None:
            self.compute_gain(v)

        state = np.array([angle_delta, offset])
        u = -self.K_lqr @ state
        return u[0]


def distance_to_point(a, b, c, x0, y0):
    # Distance function
    def distance(x):
        y = a * x**2 + b * x + c
        return np.sqrt((x - x0) ** 2 + (y - y0) ** 2)

    # Minimization using scipy's minimize function
    result = minimize(distance, 0)  # Start search at x = 0

    # Get the minimal distance and corresponding x
    x_min = result.x[0]
    min_distance = result.fun

    return x_min, min_distance


def angle_with_x_axis(a, b, x0):
    # Calculate derivative at x0
    derivative = 2 * a * x0 + b

    # Calculate angle in degrees
    angle = np.degrees(np.arctan(derivative))
    return angle


def angle_with_y_axis(a, b, x0):
    # Calculate derivative at x0
    derivative = 2 * a * x0 + b

    # Calculate angle with y-axis in degrees (90° - angle with x-axis)
    angle = 90 - np.degrees(np.arctan(derivative))
    return angle
