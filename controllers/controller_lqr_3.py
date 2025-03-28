import numpy as np
from scipy.signal import cont2discrete
from scipy.linalg import solve_discrete_are


class LQRController:
    def __init__(self, Ts=0.1, v=10.0, l=2.5, D=1.0):
        self.update_parameters(Ts, v, l, D)

    def update_parameters(self, Ts, v, l, D):
        """
        Aktualisiert Systemparameter und berechnet alle Matrizen neu.
        :param Ts: Abtastzeit
        :param v: Geschwindigkeit
        :param l: Radstand
        :param D: Fahrzeugparameter
        """
        self.Ts = Ts
        self.v = v
        self.l = l
        self.D = D

        # Kontinuierliches System
        A = np.array([[0, 0], [0, v]])

        B = np.array([[v / l], [-v * l]])

        E = np.array([[v], [-D * v / l]])

        # Diskretisierung via Zero-Order Hold
        BE = np.hstack([B, E])
        Ad, BEd, _, _, _ = cont2discrete((A, BE, np.eye(2), 0), Ts, method="zoh")
        Bd = BEd[:, [0]]
        Ed = BEd[:, [1]]

        self.Ad = Ad
        self.Bd = Bd
        self.Ed = Ed

        # LQR Design
        Q = np.diag([10.0, 1.0])
        R = np.array([[1.0]])
        P = solve_discrete_are(Ad, Bd, Q, R)
        self.K = np.linalg.inv(R + Bd.T @ P @ Bd) @ (Bd.T @ P @ Ad)

        # Feedforward-Verstärkung
        if np.all(Ed != 0):
            self.Nk = float(Bd[0, 0] / Ed[0, 0])
        else:
            self.Nk = 0.0

    def get_control_signal(self, x, k_ref):
        """
        Berechne den Regelausgang (Lenkwinkel) für den aktuellen Zustand und die Soll-Krümmung.
        :param x: np.array([Δψ, Δy])
        :param k_ref: Soll-Krümmung
        :return: Lenkwinkel in rad (float)
        """
        x = np.array(x).reshape(2, 1)
        u = -self.K @ x + self.Nk * k_ref
        return float(np.clip(u, -np.pi / 4, np.pi / 4))  # Begrenzung auf ±45°
