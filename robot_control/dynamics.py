"""Real-time rigid-body dynamics for xArm6 via PyBullet inverse dynamics.

Provides gravity, Coriolis, mass-matrix, and full inverse-dynamics torque
computation, plus a **Momentum Observer** (De Luca 2005) that estimates
external joint torques without differentiating q̇.

Typical per-call latency: ~0.013 ms (capable of >70 kHz).
"""

import numpy as np
import pybullet as pb

from .constants import N_JOINTS, PB_JOINT_INDICES, XARM6_URDF

_EPS_GRAD = 1e-4  # step size for numerical gradient of M(q)


class DynamicsModel:
    """xArm6 inverse dynamics model.

    All public methods accept angles in **degrees** and angular velocities
    in **deg/s**.  Returned torques are in **Nm**.
    """

    def __init__(self, urdf_path: str = XARM6_URDF):
        self._cid = pb.connect(pb.DIRECT)
        pb.setGravity(0, 0, -9.81, physicsClientId=self._cid)
        self._robot = pb.loadURDF(
            urdf_path, useFixedBase=True, physicsClientId=self._cid,
        )
        for j in PB_JOINT_INDICES:
            pb.changeDynamics(
                self._robot, j,
                linearDamping=0, angularDamping=0,
                physicsClientId=self._cid,
            )

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _set_q(self, q_rad):
        for i, j in enumerate(PB_JOINT_INDICES):
            pb.resetJointState(
                self._robot, j, q_rad[i], physicsClientId=self._cid,
            )

    def _id(self, q_rad, qd_rad, qdd_rad) -> np.ndarray:
        """Raw inverse dynamics call (radians in, Nm out)."""
        self._set_q(q_rad)
        tau = pb.calculateInverseDynamics(
            self._robot,
            list(q_rad), list(qd_rad), list(qdd_rad),
            physicsClientId=self._cid,
        )
        return np.array(tau[:N_JOINTS])

    def _mass_matrix_rad(self, q_rad) -> np.ndarray:
        """M(q) in radians, returns (N, N)."""
        self._set_q(q_rad)
        M = pb.calculateMassMatrix(self._robot, list(q_rad),
                                   physicsClientId=self._cid)
        return np.array(M)

    def _mdot_qd(self, q_rad, qd_rad) -> np.ndarray:
        """Ṁ(q,q̇)·q̇ via central-difference gradient of M(q).

        Ṁ = Σ_k (∂M/∂q_k)·q̇_k,  so  Ṁ·q̇ = Σ_k q̇_k · (∂M/∂q_k)·q̇
        """
        qd = np.asarray(qd_rad)
        result = np.zeros(N_JOINTS)
        for k in range(N_JOINTS):
            if abs(qd[k]) < 1e-10:
                continue
            q_plus = q_rad.copy()
            q_minus = q_rad.copy()
            q_plus[k] += _EPS_GRAD
            q_minus[k] -= _EPS_GRAD
            dM_dqk = (self._mass_matrix_rad(q_plus)
                       - self._mass_matrix_rad(q_minus)) / (2 * _EPS_GRAD)
            result += qd[k] * (dM_dqk @ qd)
        return result

    # ------------------------------------------------------------------
    # Public API (degrees / deg/s in, Nm out)
    # ------------------------------------------------------------------

    def gravity(self, q_deg: np.ndarray) -> np.ndarray:
        """G(q) — gravity torque vector."""
        q = np.deg2rad(q_deg)
        z = [0.0] * N_JOINTS
        return self._id(q, z, z)

    def coriolis(self, q_deg: np.ndarray, qd_dps: np.ndarray) -> np.ndarray:
        """C(q, q̇)·q̇ — Coriolis + centrifugal torque vector."""
        q = np.deg2rad(q_deg)
        qd = np.deg2rad(qd_dps)
        z = [0.0] * N_JOINTS
        return self._id(q, qd, z) - self._id(q, z, z)

    def mass_matrix(self, q_deg: np.ndarray) -> np.ndarray:
        """M(q) — joint-space inertia matrix (N×N)."""
        return self._mass_matrix_rad(np.deg2rad(q_deg))

    def inverse_dynamics(
        self,
        q_deg: np.ndarray,
        qd_dps: np.ndarray,
        qdd_dps2: np.ndarray,
    ) -> np.ndarray:
        """M(q)·q̈ + C(q,q̇)·q̇ + G(q) — full inverse dynamics."""
        return self._id(
            np.deg2rad(q_deg),
            np.deg2rad(qd_dps),
            np.deg2rad(qdd_dps2),
        )

    def eta(self, q_deg: np.ndarray, qd_dps: np.ndarray) -> np.ndarray:
        """η(q,q̇) = C·q̇ + G − Ṁ·q̇  (used by momentum observer)."""
        q = np.deg2rad(q_deg)
        qd = np.deg2rad(qd_dps)
        z = [0.0] * N_JOINTS
        n = self._id(q, qd, z)          # C·q̇ + G
        mdot_qd = self._mdot_qd(q, qd)  # Ṁ·q̇
        return n - mdot_qd

    def close(self):
        if pb.isConnected(self._cid):
            pb.disconnect(self._cid)


class MomentumObserver:
    """Generalized-momentum observer for external torque estimation.

    Uses integration (not differentiation) to estimate τ_ext, giving
    fast response without noise amplification.

    Reference: De Luca & Mattone, IEEE ICRA 2005.

    Parameters
    ----------
    dyn : DynamicsModel
    gain : float
        Observer gain K_O (1/s).  Higher = faster convergence.
        Time constant ≈ 1/gain.  Default 50 → ~20 ms response.
    tau_slew_max : float
        Maximum allowed rate of change for τ_meas (Nm/s).
        Rejects sudden sensor jumps (e.g. xArm J3 firmware offset switching).
    bias_model : GravityBiasModel | None
        Position-dependent bias model. If provided, bias(q) is subtracted
        from the observer output instead of a constant bias.
    """

    # Known firmware jump sizes (Nm) — "high" state adds this offset.
    # Detected joints: J2=7.534, J3=5.279, J5=3.221.
    # When a jump of this size is detected, snap it back immediately.
    _JUMP_TABLE = {
        1: 7.534,   # J2 (index 1)
        2: 5.279,   # J3 (index 2)
        4: 3.221,   # J5 (index 4)
    }
    _JUMP_TOLERANCE = 1.0  # match if within ±1 Nm of known size

    def __init__(self, dyn: DynamicsModel, gain: float = 50.0,
                 tau_slew_max: float = 50.0, bias_model=None):
        self._dyn = dyn
        self._gain = gain
        self._tau_slew_max = tau_slew_max
        self._sigma: np.ndarray | None = None
        self._tau_prev: np.ndarray | None = None
        self._tau_raw_prev: np.ndarray | None = None
        self._jump_offsets = np.zeros(N_JOINTS)
        self._bias = np.zeros(N_JOINTS)
        self._bias_model = bias_model

    def update(
        self,
        q_deg: np.ndarray,
        qd_dps: np.ndarray,
        tau_meas: np.ndarray,
        dt: float,
    ) -> np.ndarray:
        """Run one observer step.  Returns τ_ext estimate (Nm)."""
        tau_meas = tau_meas.copy()

        # Firmware jump correction: detect known fixed-size jumps and cancel
        if self._tau_raw_prev is not None:
            for idx, jump_size in self._JUMP_TABLE.items():
                delta = tau_meas[idx] - self._tau_raw_prev[idx]
                if abs(abs(delta) - jump_size) < self._JUMP_TOLERANCE:
                    # Use known jump size for exact correction
                    self._jump_offsets[idx] += np.sign(delta) * jump_size
        self._tau_raw_prev = tau_meas.copy()
        tau_meas -= self._jump_offsets

        # Slew-rate limit for remaining noise
        if self._tau_prev is not None:
            max_delta = self._tau_slew_max * dt
            delta = tau_meas - self._tau_prev
            tau_meas = self._tau_prev + np.clip(delta, -max_delta, max_delta)
        self._tau_prev = tau_meas.copy()

        q_rad = np.deg2rad(q_deg)
        qd_rad = np.deg2rad(qd_dps)

        # Generalized momentum  p = M(q) · q̇
        M = self._dyn._mass_matrix_rad(q_rad)
        p = M @ qd_rad

        # η = (C·q̇ + G) − Ṁ·q̇
        eta = self._dyn.eta(q_deg, qd_dps)

        # First call — initialise integral to current momentum
        if self._sigma is None:
            self._sigma = p.copy()
            return np.zeros(N_JOINTS)

        # Observer:  r = K · (σ − p),  σ += (τ − η − r) · dt
        r = self._gain * (self._sigma - p)
        self._sigma += (tau_meas - eta - r) * dt

        # Subtract bias: position-dependent model or constant
        if self._bias_model is not None:
            return r - self._bias_model.predict(q_deg)
        return r - self._bias

    def reset_tracking(self):
        """Reset τ_meas tracking state after motion commands."""
        self._tau_prev = None
        self._tau_raw_prev = None

    def calibrate(self, samples: list[np.ndarray]):
        """Set constant bias from calibration samples (fallback when no model)."""
        self._bias = np.mean(samples, axis=0)
        print(f"[CAL] Bias set: {['%.3f' % b for b in self._bias]}")
