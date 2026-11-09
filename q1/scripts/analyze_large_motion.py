"""Analyze recorded Q1 motion only; never connects to or commands the robot."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def stats(values):
    x = np.asarray(values)
    return dict(n=int(x.size), mean=float(x.mean()), p95=float(np.percentile(x, 95)),
                maximum=float(x.max())) if x.size else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--plot', type=Path, required=True)
    args = parser.parse_args()
    episodes = json.loads((args.root / 'input-and-tracking-audit.json').read_text())
    contract = json.loads((args.model / 'deployment_contract.json').read_text())
    names = [j['name'] for j in contract['joints']]
    output = []
    for episode in episodes:
        series = episode['series']
        t = np.array([r['t'] for r in series])
        frame = np.array([r['frame'] for r in series])
        q, model, target, motor = [np.array([r[k] for r in series])
                                   for k in ['q', 'model_q', 'target', 'motor']]
        stages = [r['stages'] for r in series]
        before = np.array([s['before_tracking_q'] for s in stages])
        after = np.array([s['after_tracking_q'] for s in stages])
        tracking = np.abs(before - after) > 1e-6
        mechanical = np.abs(model - target) > 1e-5
        age = np.array([s['observation_age_ms'] for s in stages])
        reference = np.load(args.model / 'references_50hz' / (episode['clip'] + '.npz'))
        goals, reference_q = reference['goal_590'], reference['q']
        ref = reference_q[frame]
        errors = sum(hashlib.sha256(np.asarray(goals[h['frame']], dtype='<f4').tobytes()).hexdigest()
                     != h['goal_hash'] for h in episode['goal_hashes'])
        rows = []
        for begin, end in [(0, 8), (8, 12), (12, 16), (16, 100)]:
            mask = (t >= begin) & (t < end)
            if not mask.any():
                continue
            errors_by_joint = []
            for j, name in enumerate(names):
                errors_by_joint.append(dict(
                    name=name,
                    reference_mean_rad=float(ref[mask, j].mean()),
                    native_policy_mean_rad=float(model[mask, j].mean()),
                    hal_command_mean_rad=float(motor[mask, j].mean()),
                    measured_mean_rad=float(q[mask, j].mean()),
                    native_policy_to_hal_mae=float(np.abs(model[mask, j] - motor[mask, j]).mean()),
                    policy_to_actual_mae=float(np.abs(target[mask, j] - q[mask, j]).mean()),
                    hal_to_actual_mae=float(np.abs(motor[mask, j] - q[mask, j]).mean()),
                    reference_to_actual_rmse=float(np.sqrt(np.mean((ref[mask, j] - q[mask, j])**2))),
                    mechanical_clip_fraction=float(mechanical[mask, j].mean()),
                    tracking_clip_fraction=float(tracking[mask, j].mean()),
                ))
            rows.append(dict(
                time_window=[begin, min(end, float(t.max()))], count=int(mask.sum()),
                reference_frame_range=[int(frame[mask].min()), int(frame[mask].max())],
                mechanical_clip_fraction=float(mechanical[mask].any(axis=1).mean()),
                tracking_clip_fraction=float(tracking[mask].any(axis=1).mean()),
                observation_age_ms=stats(age[mask]),
                legs_policy_tracking_mae=float(np.abs(target[mask, :12] - q[mask, :12]).mean()),
                arms_policy_tracking_mae=float(np.abs(target[mask, 14:] - q[mask, 14:]).mean()),
                joints=errors_by_joint,
            ))
        onset = []
        for j, name in enumerate(names):
            hits = np.flatnonzero(tracking[:, j])
            if hits.size:
                k = int(hits[0])
                onset.append(dict(name=name, first_time=float(t[k]), reference_frame=int(frame[k]),
                                  measured_q=float(q[k, j]), policy_target=float(target[k, j]),
                                  hal_target=float(motor[k, j]), preceding_error_rad=[
                                      float(target[m, j] - q[m, j]) for m in range(max(0, k-4), k+1)]))
        # These comparisons are diagnostics, not evidence that a policy target must equal reference q.
        output.append(dict(episode=episode['episode'], clip=episode['clip'], goal_hash_errors=errors,
                           checks=episode['checks'], windows=rows, tracking_limit_first_activation=onset,
                           notes=['Reference q is a motion goal; policy targets include PD corrections.',
                                  'All samples are real logged robot observations and commands.',
                                  'Phone video does not provide a synchronized control timestamp.']))
        if episode is episodes[-1]:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            fig, axes = plt.subplots(6, 1, figsize=(13, 15), sharex=True)
            for ax, j in zip(axes[:5], [1, 7, 4, 12, 17]):
                ax.plot(t, ref[:, j], color='#888888', label='Reference pose', linewidth=1)
                ax.plot(t, model[:, j], color='#b56627', label='Native policy target', linewidth=1)
                ax.plot(t, motor[:, j], color='#8a52bf', label='HAL command', linewidth=1)
                ax.plot(t, q[:, j], color='#047c86', label='Measured joint', linewidth=1.3)
                ax.scatter(t[tracking[:, j]], motor[tracking[:, j], j], s=9, color='#d12d44',
                           label='Tracking limiter active', zorder=4)
                ax.set_title(names[j]); ax.set_ylabel('rad'); ax.grid(alpha=.25)
            axes[0].legend(loc='upper left', fontsize=8, ncol=3)
            axes[-1].plot(t, np.abs(target-q).max(axis=1), label='Max policy / joint error', color='#047c86')
            axes[-1].axhline(.5, color='#d12d44', linestyle='--', label='Tracking correction threshold')
            axes[-1].set_ylabel('rad'); axes[-1].grid(alpha=.25)
            twin = axes[-1].twinx(); twin.plot(t, age, color='#d4a94d', alpha=.45)
            twin.set_ylabel('Observation to HAL age (ms)')
            axes[-1].legend(loc='upper left', fontsize=8)
            axes[-1].set_xlabel('Seconds after onboard play request (not phone video time)')
            fig.suptitle('Recorded Q1 | dance1_subject2 | previous action fix deployed')
            fig.tight_layout(); args.plot.parent.mkdir(parents=True, exist_ok=True)
            fig.savefig(args.plot, dpi=140); plt.close(fig)
    (args.root / 'segmented-assessment.json').write_text(json.dumps(output, indent=2) + '\n')
    for episode in output:
        print(json.dumps({**{k: episode[k] for k in ['episode', 'goal_hash_errors', 'checks']},
                          'windows': [{k: v for k, v in w.items() if k != 'joints'}
                                      for w in episode['windows']]}, ensure_ascii=False))


if __name__ == '__main__':
    main()
