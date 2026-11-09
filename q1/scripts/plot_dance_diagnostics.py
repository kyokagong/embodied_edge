"""Audit actual dance input against release NPZ and plot actual logged trajectories."""
import argparse,hashlib,json,math
from pathlib import Path
import numpy as np


def main():
    p=argparse.ArgumentParser();p.add_argument('--trace',type=Path,required=True);p.add_argument('--reference',type=Path,required=True);p.add_argument('--report',type=Path,required=True);p.add_argument('--plot',type=Path);a=p.parse_args()
    trace=json.loads(a.trace.read_text());ref=np.load(a.reference);h=trace['hardware'];home=np.array(h['stand_trial']['home']);res=np.array(h['stand_trial']['policy_residual']) if h['stand_trial'].get('policy_residual') is not None else np.maximum(np.abs(np.array(h['lower'])-home),np.abs(np.array(h['upper'])-home));lower=np.maximum(home-res,h['lower']);upper=np.minimum(home+res,h['upper'])
    goals=ref['goal_590'];rotations=ref['root_R'];qdata=ref['q']
    mismatch=[];frames=[]
    for i in trace['inputs']:
        f=i['frame'];frames.append(f)
        if hashlib.sha256(np.asarray(goals[f],dtype='<f4').tobytes()).hexdigest()!=i['goal_sha256']:mismatch.append(dict(id=i['id'],frame=f,field='goal'))
        if hashlib.sha256(np.asarray(rotations[f],dtype='<f4').tobytes()).hexdigest()!=i['reference_rotation_sha256']:mismatch.append(dict(id=i['id'],frame=f,field='reference_rotation'))
    qref=qdata[frames];series=trace['series'];indices=[x['frame'] for x in series];qa=np.array([x['actual_q'] for x in series]);qs=np.array([x['sent_q'] for x in series]);qm=np.array([x['model_q'] for x in series]);qr=qdata[indices]
    catalog=json.load(open(a.reference.parents[1]/'dance_catalog.json'));clip=next(x for x in catalog['clips'] if x['id']=='dance1_subject1')
    contract=json.load(open(a.reference.parents[1]/'deployment_contract.json'));names=[j['name'] for j in contract['joints']]
    summaries=[]
    for j in range(22):
        inside=(qref[:,j]>=lower[j])&(qref[:,j]<=upper[j]);e=qa[:,j]-qr[:,j]
        summaries.append(dict(name=names[j],reference_range_rad=[float(qref[:,j].min()),float(qref[:,j].max())],allowed_position_range_rad=[float(lower[j]),float(upper[j])],reference_outside_added_amplitude_fraction=float(np.mean(~inside)),reference_to_actual_rmse_rad=float(np.sqrt(np.mean(e**2))),sent_to_actual_mae_rad=float(np.mean(np.abs(qs[:,j]-qa[:,j]))),model_target_to_sent_mean_change_rad=float(np.mean(np.abs(qm[:,j]-qs[:,j])))))
    report=dict(source='real_Q1_input_hashes_and_actual_kinematic_feedback',goal_or_rotation_hash_mismatches=mismatch,matched_input_count=len(frames),reference_first_frame=min(frames),reference_last_frame=max(frames),unique_reference_frames=len(set(frames)),total_clip_frames=clip['frames_50hz'],total_clip_duration_s=clip['duration_s'],checks=trace['checks'],yaw_initial_deg=trace['yaw_initial_deg'],orientation_error_deg=trace['orientation_error_deg'],joints=summaries)
    a.report.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');a.report.chmod(0o600)
    print(json.dumps({k:report[k] for k in report if k!='joints'},ensure_ascii=False));print(json.dumps([x for x in summaries if 'elbow' in x['name'] or 'shoulder_pitch' in x['name']],ensure_ascii=False))
    if a.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,axes=plt.subplots(3,1,figsize=(12,10),sharex=True);t=[x['t'] for x in series]
        for ax,j in zip(axes,[17,21,7]):
            ax.fill_between(t,lower[j],upper[j],color='#e8a838',alpha=.15,label='Allowed extra amplitude')
            ax.plot(t,qr[:,j],color='#626b76',lw=1.3,label='Reference pose (not a direct PD command)')
            ax.plot(t,qm[:,j],color='#d45a55',lw=1,alpha=.9,label='Policy target before trial amplitude cap')
            ax.plot(t,qs[:,j],color='#e0a000',lw=1.5,label='Sent policy target')
            ax.plot(t,qa[:,j],color='#167c91',lw=1,label='Measured joint position')
            ax.set_ylabel('rad');ax.set_title(names[j]);ax.grid(alpha=.2)
        axes[-1].set_xlabel('Seconds after play request (robot monotonic clock)');axes[0].legend(fontsize=8,ncol=2,loc='best');fig.suptitle('dance1_subject1 | actual Q1 logs | extra amplitude clipping remains')
        fig.tight_layout();fig.savefig(a.plot,dpi=160);plt.close(fig)


if __name__=='__main__':main()
