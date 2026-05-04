"""Position-dependent gravity bias model for xArm6.

Fits a linear model in trigonometric features of joint angles to predict
the systematic τ_ext bias caused by URDF mass/COM parameter mismatch.

Usage:
    model = GravityBiasModel.load("assets/gravity_bias_model.npz")
    bias = model.predict(q_deg)  # (6,) array
"""

import numpy as np

from .constants import N_JOINTS

# Default model path relative to package root
_DEFAULT_MODEL = "assets/gravity_bias_model.npz"


class GravityBiasModel:
    """Per-joint bias predictor using trigonometric features of q."""

    def __init__(self):
        self._coefs: list[np.ndarray] | None = None
        self._residuals: np.ndarray | None = None

    @staticmethod
    def _features(q_deg: np.ndarray) -> np.ndarray:
        """Build feature vector with single-joint and coupled-angle terms.

        Gravity torques are trigonometric functions of cumulative joint angles,
        so we include sin/cos of individual joints AND coupled sums like
        q2+q3, q2+q3+q4, q2+q3+q5 to capture the kinematic chain.
        """
        q = np.deg2rad(q_deg[:N_JOINTS])
        feats = [1.0]
        # Single-joint terms
        for i in range(N_JOINTS):
            feats.extend([np.sin(q[i]), np.cos(q[i])])
        # Coupled angle sums (gravity chain)
        feats.extend([np.sin(q[1]+q[2]), np.cos(q[1]+q[2])])
        feats.extend([np.sin(q[1]+q[2]+q[3]), np.cos(q[1]+q[2]+q[3])])
        feats.extend([np.sin(q[1]+q[2]+q[4]), np.cos(q[1]+q[2]+q[4])])
        # Cross terms
        feats.extend([np.sin(q[1])*np.sin(q[2]), np.cos(q[1])*np.cos(q[2])])
        feats.extend([np.sin(q[2])*np.sin(q[4]), np.cos(q[2])*np.cos(q[4])])
        return np.array(feats)  # length = 23

    def fit(self, q_data: np.ndarray, tau_ext_data: np.ndarray,
            alpha: float = 1.0):
        """Fit per-joint ridge regression model from calibration data.

        Parameters
        ----------
        q_data : (N, 6) joint angles in degrees at each calibration pose
        tau_ext_data : (N, 6) mean τ_ext at each pose (the bias to model)
        alpha : float
            Ridge regularization strength (default: 1.0). Prevents
            overfitting when samples/features ratio is low.
        """
        X = np.array([self._features(q) for q in q_data])
        n_feat = X.shape[1]
        self._coefs = []
        self._residuals = np.zeros(N_JOINTS)
        for j in range(N_JOINTS):
            y = tau_ext_data[:, j]
            # Ridge regression: (X'X + αI)w = X'y
            A = X.T @ X + alpha * np.eye(n_feat)
            b = X.T @ y
            w = np.linalg.solve(A, b)
            self._coefs.append(w)
            y_pred = X @ w
            self._residuals[j] = np.sqrt(np.mean((y - y_pred) ** 2))

    def predict(self, q_deg: np.ndarray) -> np.ndarray:
        """Predict bias vector for a given pose."""
        if self._coefs is None:
            return np.zeros(N_JOINTS)
        x = self._features(q_deg)
        return np.array([w @ x for w in self._coefs])

    def save(self, path: str):
        """Save fitted model to .npz file."""
        if self._coefs is None:
            raise RuntimeError("Model not fitted yet")
        np.savez(
            path,
            coefs=np.array(self._coefs),
            residuals=self._residuals,
        )

    @classmethod
    def load(cls, path: str) -> "GravityBiasModel":
        """Load a fitted model from .npz file."""
        data = np.load(path)
        m = cls()
        m._coefs = list(data["coefs"])
        m._residuals = data["residuals"]
        return m

    def summary(self):
        """Print model fit quality."""
        if self._residuals is None:
            print("Model not fitted.")
            return
        print("Gravity bias model — RMSE per joint:")
        for j in range(N_JOINTS):
            print(f"  J{j+1}: {self._residuals[j]:.4f} Nm")
