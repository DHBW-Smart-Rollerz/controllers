import numpy as np
from scipy.linalg import solve_discrete_are
from scipy.signal import cont2discrete


class LQRController:
    def __init__(self, Ts=0.005, v=1.00, l=0.27, D=0.2):
        self.xi = 0.0  # Integratorzustand
        self.update_parameters(Ts, v, l, D)

    def update_parameters(self, Ts, v, l, D):
        """
        Aktualisiert die Systemparameter und berechnet alle Regler-Matrizen neu.
        """
        self.Ts = Ts
        self.v = v
        self.l = l
        self.D = D

        # Modellabhängige Parameter
        lh = 0.07
        k_psi = v / 0.271

        # Systemmatrizen
        A = np.array([[0.0, 0.0], [v, 0.0]])

        B = np.array([[k_psi], [-D * k_psi + (lh * k_psi) / v]])

        C = np.array([[1.0, 0.0]])  # Ausgang: y = Cx

        # Erweiterte Systemdarstellung
        A_ext = np.block([[A, np.zeros((2, 1))], [-C, np.zeros((1, 1))]])
        B_ext = np.vstack([B, [[0.0]]])

        # Diskretisierung
        C_dummy = np.zeros((1, 3))
        D_dummy = np.zeros((1, 1))
        system_d = cont2discrete((A_ext, B_ext, C_dummy, D_dummy), Ts)
        Ad, Bd, _, _, _ = system_d

        self.Ad = Ad
        self.Bd = Bd

        # LQR-Auslegung
        Q = np.diag([20.0, 50.0, 10])  # Zustände: Δψ, Δy, xi
        R = np.array([[50]])

        P = solve_discrete_are(Ad, Bd, Q, R)
        K_total = np.linalg.inv(Bd.T @ P @ Bd + R) @ (Bd.T @ P @ Ad)

        self.Kx = K_total[0, :2]  # Rückführung auf [Δψ, Δy]
        self.Ki = K_total[0, 2]  # Rückführung auf xi

    def reset(self):
        """Setzt den internen Integratorzustand zurück."""
        self.xi = 0.0

    def get_control_signal(self, x, y_ref=0.0):
        """
        Liefert das Steuersignal u in Abhängigkeit vom aktuellen Zustand x und Sollwert y_ref.
        """
        x = np.array(x).reshape(2, 1)
        y = float(np.dot(np.array([[1.0, 0.0]]), x))  # y = C x

        # Fehler und Integration
        e = y_ref - y
        self.xi = self.Ts * e

        # Steuerbefehl
        u = -self.Kx @ x.flatten() - self.Ki * self.xi
        return float(np.clip(u, -np.pi / 4, np.pi / 4))  # Begrenzung auf ±45°
