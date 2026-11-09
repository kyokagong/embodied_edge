"""Kinematic consistency estimates from actual logs, not measured foot contacts."""
import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np


def quat(wxyz):
    w, x, y, z = np.asarray(wxyz) / np.linalg.norm(wxyz)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def numbers(element, key, default):
    return np.array([float(v) for v in element.get(key, default).split()])


def axis_rotation(axis, angle):
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    k = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle)*k + (1-np.cos(angle))*(k@k)


def fk(body, angles):
    result = {body.get('name'): (np.eye(3), np.zeros(3))}

    def descend(parent, rotation, position):
        for child in parent.findall('body'):
            r = rotation @ quat(numbers(child, 'quat', '1 0 0 0'))
            p = position + rotation @ numbers(child, 'pos', '0 0 0')
            for joint in child.findall('joint'):
                anchor = numbers(joint, 'pos', '0 0 0')
                dr = axis_rotation(numbers(joint, 'axis', '0 0 1'), angles[joint.get('name')])
                p = p + r @ (anchor-dr@anchor)
                r = r @ dr
            result[child.get('name')] = (r, p)
            descend(child, r, p)
    descend(body, np.eye(3), np.zeros(3))
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--audit', type=Path, required=True)
    p.add_argument('--model', type=Path, required=True)
    p.add_argument('--xml', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    c = json.loads((a.model/'deployment_contract.json').read_text())
    names = [x['name'] for x in c['joints']]
    series = json.loads(a.audit.read_text())[-1]['series']
    root = ET.parse(a.xml).getroot().find('worldbody/body')
    candidates = ['pelvis', 'waist_roll_link', 'torso_link', 'head_link']
    data = []
    ref = np.load(a.model/'references_50hz/dance1_subject2.npz')
    refq, refR = ref['q'], ref['root_R']
    for sample in series:
        q = dict(zip(names, sample['q']))
        links = fk(root, q)
        xyzw = sample['quat']
        rim = quat([xyzw[3], *xyzw[:3]])
        estimates = {}
        for candidate in candidates:
            if candidate not in links:
                continue
            # Hypothesis: sensor axes match this link, with no mounting offset.
            rp = rim @ links[candidate][0].T
            tilt = []
            feet = []
            for side in ['left', 'right']:
                rf, pf = links[side+'_ankle_roll_link']
                tilt.append(float(np.degrees(np.arccos(np.clip((rp@rf)[2, 2], -1, 1)))))
                feet.append(rp@pf)
            estimates[candidate] = dict(sole_tilt_deg=tilt,
                foot_origin_height_difference_m=float(feet[0][2]-feet[1][2]),
                horizontal_foot_separation_m=float(np.linalg.norm((feet[0]-feet[1])[:2])))
        frame = sample['frame']
        rl = fk(root, dict(zip(names, refq[frame])))
        reference_feet = [refR[frame] @ rl[side+'_ankle_roll_link'][1] for side in ['left','right']]
        data.append(dict(t=sample['t'], frame=frame, candidate_frames=estimates,
            reference_foot_origin_height_difference_m=float(reference_feet[0][2]-reference_feet[1][2]),
            reference_horizontal_foot_separation_m=float(np.linalg.norm((reference_feet[0]-reference_feet[1])[:2]))))
    summary = []
    for lo, hi in [(0, 2), (2, 8), (8, 12), (12, 16), (16, 19)]:
        z = [r for r in data if lo <= r['t'] < hi]
        by_candidate = {}
        for candidate in candidates:
            if candidate not in z[0]['candidate_frames']:
                continue
            f = [r['candidate_frames'][candidate] for r in z]
            by_candidate[candidate] = dict(
                mean_sole_tilt_deg=np.mean([r['sole_tilt_deg'] for r in f], axis=0).tolist(),
                mean_abs_foot_origin_height_difference_m=float(np.mean([abs(r['foot_origin_height_difference_m']) for r in f])),
                mean_horizontal_foot_separation_m=float(np.mean([r['horizontal_foot_separation_m'] for r in f])))
        summary.append(dict(time_window=[lo,hi], count=len(z), candidates=by_candidate,
            reference_abs_foot_origin_height_difference_m=float(np.mean([abs(r['reference_foot_origin_height_difference_m']) for r in z])),
            reference_horizontal_foot_separation_m=float(np.mean([r['reference_horizontal_foot_separation_m'] for r in z]))))
    a.output.write_text(json.dumps(dict(
        source='Actual recorded joint angles and IMU quaternion; reference release XML kinematics.',
        limitation='Foot positions/contact forces are not measured. Flat-foot and mounting hypotheses are unverified; this is a consistency estimate, not proof of IMU location or contact.',
        summary=summary, series=data), indent=2)+'\n')
    print(json.dumps(summary))


if __name__ == '__main__':
    main()
