import numpy as np
from scipy.linalg import expm, solve_discrete_are


class LQRController:
    def __init__(self, Ts=0.001, v=0.2, l=0.7, D=0):
        self.Ts = Ts
        self.v = v
        self.l = l
        self.D = D

        # Systemmatrizen (kontinuierlich)
        A = np.array([[0, 0], [0, v]])
        B = np.array([[v / l], [-v]])
        E = np.array([[v], [-D * v / l]])

        # Diskretisierung (ähnlich MATLAB)
        self.Ad = expm(A * Ts)
        Phi_int = self._compute_phi_integral(A, Ts)
        self.Bd = Phi_int @ B
        self.Ed = Phi_int @ E

        # LQR Design
        Q = np.diag([10, 30])
        R = np.array([[10000]])

        P = solve_discrete_are(self.Ad, self.Bd, Q, R)
        self.K = np.linalg.inv(R + self.Bd.T @ P @ self.Bd) @ (self.Bd.T @ P @ self.Ad)

        # Feedforward-Anteil
        self.Nk = -np.linalg.pinv(self.Bd) @ self.Ed

    def _compute_phi_integral(self, A, Ts):
        # Approximation der Integralexponentialmatrix
        steps = 100
        dt = Ts / steps
        phi_sum = np.zeros_like(A)
        for i in range(steps):
            tau = i * dt
            phi_sum += expm(A * tau) * dt
        return phi_sum

    def get_control_signal(self, x, k):
        """
        :param x: Zustand [DeltaPsi, DeltaY]
        :param k: Zielkrümmung
        :return: Lenkwinkel (in Grad, begrenzt auf ±45)
        """
        x = np.array(x).reshape(2, 1)
        u = -self.K @ x  # + self.Nk * k
        u = np.clip(u, -np.pi / 4, np.pi / 4)  # Begrenzung ±45°
        return int(np.degrees(u.item()))
