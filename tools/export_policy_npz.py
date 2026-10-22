#!/usr/bin/env python3
"""Export the deterministic actor of an SB3 policy to .npz for helperFunction/haptic_policy.MlpPolicy.

Run in the suctionRL conda env (torch + SB3), e.g.
  ~/miniconda3/envs/suctionRL/bin/python tools/export_policy_npz.py --all
--all exports the three robot-experiment policies listed in ROBOT_POLICIES (paths in 2D_Haptic_Search/outputs).
"""
import argparse
import hashlib
from pathlib import Path

import numpy as np

REPO = Path('~/projects/2D_Haptic_Search').expanduser()
PACKAGE = Path(__file__).resolve().parents[1]
ROBOT_POLICIES = {
    'ppo_ch3_s2_ch3_s5': ('ppo', 'outputs/v2_s234_20261019/runs/s2_ch3_s5/final.zip'),
    'ppo_ch6_s2_ch6_s1': ('ppo', 'outputs/v2_s234_20261019/runs/s2_ch6_s1/final.zip'),
    'sac_ch3_ch3_sac_log_s3': ('sac', 'outputs/v2_s1_confirm_20261019/runs/ch3_sac_log_s3/final.zip'),
}


def linear_stack(sequential, head):
    layers = [m for m in sequential if hasattr(m, 'weight')] + [head]
    acts = {type(m).__name__ for m in sequential if not hasattr(m, 'weight')}
    if len(acts) != 1:
        raise ValueError(f'Expected one activation type, found {acts}')
    return layers, {'Tanh': 'tanh', 'ReLU': 'relu'}[acts.pop()]


def export(algo, zip_path, out_path, variant='mlp_h5s5_log'):
    from stable_baselines3 import PPO, SAC
    model = (PPO if algo == 'ppo' else SAC).load(zip_path, device='cpu')
    if algo == 'ppo':
        policy = model.policy
        if policy.squash_output:
            raise ValueError('Squashed PPO policies are not supported')
        layers, activation = linear_stack(policy.mlp_extractor.policy_net, policy.action_net)
        squash = False
    else:
        layers, activation = linear_stack(model.policy.actor.latent_pi, model.policy.actor.mu)
        squash = True
    low, high = model.action_space.low, model.action_space.high
    if not (np.all(low == -1) and np.all(high == 1)):
        raise ValueError('Expected the [-1, 1] action box')
    obs_dim = model.observation_space.shape[0]
    chambers = obs_dim // 5 - 4
    if obs_dim != 5 * (chambers + 4):
        raise ValueError(f'Observation size {obs_dim} does not match mlp_h5s5 history')
    arrays = {f'W{i}': l.weight.detach().numpy().astype(np.float32) for i, l in enumerate(layers)}
    arrays.update({f'b{i}': l.bias.detach().numpy().astype(np.float32) for i, l in enumerate(layers)})
    np.savez(out_path, algo=algo, chambers=chambers, variant=variant, activation=activation, squash=squash,
             n_layers=len(layers), source=str(zip_path),
             source_sha256=hashlib.sha256(Path(zip_path).read_bytes()).hexdigest(), **arrays)
    print(f'{out_path}: {algo} {chambers}ch {activation} squash={squash} layers={[a.shape for k, a in arrays.items() if k[0] == "W"]}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--all', action='store_true', help='export ROBOT_POLICIES into policies/')
    ap.add_argument('--algo', choices=('ppo', 'sac'))
    ap.add_argument('--zip')
    ap.add_argument('--out')
    args = ap.parse_args()
    if args.all:
        for name, (algo, rel) in ROBOT_POLICIES.items():
            export(algo, REPO / rel, PACKAGE / 'policies' / f'{name}.npz')
    elif args.algo and args.zip and args.out:
        export(args.algo, args.zip, args.out)
    else:
        ap.error('use --all or give --algo, --zip and --out')


if __name__ == '__main__':
    main()
