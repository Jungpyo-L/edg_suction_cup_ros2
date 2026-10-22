#!/usr/bin/env python3
"""Continuous haptic-search controllers that reproduce the 2D_Haptic_Search simulator exactly (numpy only).

Matches haptic_search/continuous/rl_env.py (variant mlp_h5s5_log, pressure_model v2) and
haptic_search/continuous/controllers.py as used by tools/evaluate_v2_final.py. No ROS imports, so the same
module runs in the ROS2 nodes (system Python) and in the parity test (suctionRL env with the simulator).

Conventions (all in the simulator's frames; the robot mapping lives elsewhere):
- pressures: chamber vacuum in Pa, POSITIVE when sucking, in simulator chamber order
  (chamber i centred at 180/n + 360 i/n deg in the cup body frame, counter-clockwise seen from above).
- action: applied body-frame command in [-1, 1]^3 with |a[:2]| <= 1;
  planar velocity = R(yaw) a[:2] * SPEED_MM_S in the world, yaw rate = a[2] * YAW_RATE_DEG_S (CCW from above).
- one call to act() is one 50 Hz control tick.
"""
from collections import deque

import numpy as np

P_MAX_PA = 64400.0
OBS_REF_PA = 10.0
CONTROL_HZ = 50.0
SPEED_MM_S = 10.0
YAW_RATE_DEG_S = 10.0
DURATION_S = 10.0
HISTORY = 5
STRIDE = 5
HEURISTIC_TAU_S = 0.5
HEURISTIC_MODES = ('greedy', 'yaw', 'momentum', 'yaw_momentum')


def normalize_log(pressures_pa):
    """pressure_v2.normalize_log: log1p(p / 10 Pa) / log1p(64400 / 10), clipped to [0, 1]."""
    p = np.clip(np.asarray(pressures_pa, dtype=float), 0., None)
    return np.clip(np.log1p(p / OBS_REF_PA) / np.log1p(P_MAX_PA / OBS_REF_PA), 0., 1.)


def limit_action(action):
    """rl_env.step: clip to [-1, 1], then scale the planar part down to unit norm if it is longer."""
    action = np.asarray(action, dtype=float)
    if action.shape != (3,) or not np.all(np.isfinite(action)):
        raise ValueError('Expected a finite action of shape (3,)')
    applied = np.clip(action, -1., 1.)
    applied[:2] /= max(1., float(np.linalg.norm(applied[:2])))
    return applied


def chamber_unit_vectors(chambers, yaw_deg=0.):
    """physics_model.calculate_unit_vectors: chamber i points at 180/n + 360 i/n + yaw degrees."""
    angles = np.pi / chambers + 2 * np.pi * np.arange(chambers) / chambers + np.radians(yaw_deg)
    return np.stack([np.cos(angles), np.sin(angles)], axis=1)


def rotation(yaw_deg):
    a = np.radians(yaw_deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s], [s, c]])


class MlpPolicy:
    """Deterministic actor exported from SB3 by tools/export_policy_npz.py.

    PPO (gSDE, no squash): action_net(latent), clipped to the action box by predict().
    SAC: tanh(mu(latent)); the action box is [-1, 1] so unscaling is the identity.
    """

    def __init__(self, path):
        data = np.load(path, allow_pickle=False)
        self.algo = str(data['algo'])
        self.chambers = int(data['chambers'])
        self.variant = str(data['variant'])
        self.activation = str(data['activation'])
        self.squash = bool(data['squash'])
        self.source_sha256 = str(data['source_sha256'])
        n_layers = int(data['n_layers'])
        self.weights = [data[f'W{i}'].astype(np.float32) for i in range(n_layers)]
        self.biases = [data[f'b{i}'].astype(np.float32) for i in range(n_layers)]
        if self.activation not in ('tanh', 'relu'):
            raise ValueError(f'Unsupported activation {self.activation}')
        if self.variant != 'mlp_h5s5_log':
            raise ValueError(f'Observation builder only implements mlp_h5s5_log, not {self.variant}')

    def __call__(self, obs):
        x = np.asarray(obs, dtype=np.float32)
        last = len(self.weights) - 1
        for i, (W, b) in enumerate(zip(self.weights, self.biases)):
            x = W @ x + b
            if i < last:
                x = np.tanh(x) if self.activation == 'tanh' else np.maximum(x, 0.)
        if self.squash:
            x = np.tanh(x)
        return np.clip(x, -1., 1.)


class PolicyController:
    """Observation history of rl_env (mlp_h5s5_log) around an exported policy.

    A frame is [normalize_log(p) (n), mean of the last STRIDE applied actions (3), 1.0]; the observation is the
    frames at lags 20, 15, 10, 5, 0 ticks (oldest first), zero-padded before the episode start.
    """

    def __init__(self, policy_path, chambers=None):
        self.policy = MlpPolicy(policy_path)
        if chambers is not None and chambers != self.policy.chambers:
            raise ValueError(f'Policy was trained for {self.policy.chambers} chambers, not {chambers}')
        self.chambers = self.policy.chambers
        self.reset()

    def reset(self):
        self.frames = deque(maxlen=(HISTORY - 1) * STRIDE + 1)
        self.actions = deque(maxlen=STRIDE)
        self.last_obs = None

    def observe(self, pressures_pa):
        """Append the frame for this tick and return the observation, without choosing an action."""
        p = np.asarray(pressures_pa, dtype=float)
        if p.shape != (self.chambers,) or not np.all(np.isfinite(p)):
            raise ValueError(f'Expected {self.chambers} finite pressures')
        avg = np.mean(self.actions, axis=0) if self.actions else np.zeros(3)
        self.frames.append(np.r_[normalize_log(p), avg, 1.].astype(np.float32))
        frames, zero = list(self.frames), np.zeros(self.chambers + 4, np.float32)
        self.last_obs = np.concatenate([frames[len(frames) - 1 - lag] if lag < len(frames) else zero
                                        for lag in reversed(range(0, HISTORY * STRIDE, STRIDE))])
        return self.last_obs

    def act(self, pressures_pa, yaw_deg=None, dt=None):
        """One control tick: vacuum pressures (Pa, positive) -> applied body-frame action. yaw/dt are unused."""
        applied = limit_action(self.policy(self.observe(pressures_pa)))
        self.actions.append(applied.copy())
        return applied


class HeuristicController:
    """controllers.ContinuousHeuristic(mode, n, 10 mm/s, 10 deg/s, tau 0.5 s) converted to an applied action
    exactly as tools/evaluate_v2_final.py does: a = [R(-yaw) v / SPEED, w / YAW_RATE], then limit_action.

    The momentum filter runs in the world frame, so it needs yaw. Only yaw changes matter: adding a constant to
    every yaw rotates the filtered world velocity with it and leaves the body-frame action unchanged, so the
    robot can pass yaw measured relative to the trial start (with the simulator's sign).
    """

    def __init__(self, mode, chambers, speed_mm_s=SPEED_MM_S, yaw_rate_deg_s=YAW_RATE_DEG_S, tau_s=HEURISTIC_TAU_S):
        if mode not in HEURISTIC_MODES:
            raise ValueError(f'Unknown heuristic {mode}')
        self.mode, self.chambers = mode, chambers
        self.speed, self.yaw_rate, self.tau = speed_mm_s, yaw_rate_deg_s, tau_s
        self.reset()

    def reset(self):
        self.velocity = np.zeros(2)

    def act(self, pressures_pa, yaw_deg, dt=1. / CONTROL_HZ):
        p = np.asarray(pressures_pa, dtype=float)
        if p.shape != (self.chambers,) or not np.all(np.isfinite(p)) or not np.isfinite(yaw_deg):
            raise ValueError(f'Expected {self.chambers} finite pressures and a finite yaw')
        vec = p @ chamber_unit_vectors(self.chambers, yaw_deg)
        norm = np.linalg.norm(vec)
        direction = vec / norm if np.isfinite(norm) and norm >= 1e-12 else np.zeros(2)
        target = self.speed * direction
        if 'momentum' in self.mode:
            alpha = np.exp(-dt / self.tau)
            self.velocity = alpha * self.velocity + (1 - alpha) * target
        else:
            self.velocity = target
        w = self.yaw_rate if 'yaw' in self.mode else 0.
        return limit_action(np.r_[rotation(-yaw_deg) @ self.velocity / SPEED_MM_S, w / YAW_RATE_DEG_S])


def make_controller(controller, chambers, policy_path=None):
    """'ppo'/'sac' need policy_path (an exported .npz); anything else is a heuristic mode."""
    if controller in ('ppo', 'sac'):
        if policy_path is None:
            raise ValueError(f'{controller} needs an exported policy .npz')
        ctrl = PolicyController(policy_path, chambers)
        if ctrl.policy.algo != controller:
            raise ValueError(f'{policy_path} holds a {ctrl.policy.algo} policy, not {controller}')
        return ctrl
    return HeuristicController(controller, chambers)
