"""Summarize actual Q1 runs against the exact release reference; no simulation."""
import argparse,hashlib,json
from pathlib import Path
import numpy as np


def main():
 p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--model',type=Path,required=True);p.add_argument('--plot',type=Path,required=True);a=p.parse_args()
 data=json.load(open(a.root/'input-and-tracking-audit.json'));contract=json.load(open(a.model/'deployment_contract.json'));names=[x['name'] for x in contract['joints']];out=[]
 import matplotlib;matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 fig,axes=plt.subplots(3,1,figsize=(12,9),sharex=True)
 for e in data:
  ref=np.load(a.model/'references_50hz'/str(e['clip']+'.npz'));goals=ref['goal_590'];q=ref['q'];errors=sum(hashlib.sha256(np.asarray(goals[x['frame']],dtype='<f4').tobytes()).hexdigest()!=x['goal_hash'] for x in e['goal_hashes']);series=e['series'];frames=[x['frame'] for x in series];actual=np.array([x['q'] for x in series]);target=np.array([x['target'] for x in series]);reference=q[frames]
  metrics=[]
  for i,name in enumerate(names):
   metrics.append(dict(name=name,reference_to_actual_rmse_rad=float(np.sqrt(np.mean((actual[:,i]-reference[:,i])**2))),target_to_actual_mae_rad=float(np.mean(np.abs(actual[:,i]-target[:,i])))))
  checks=e['checks'];out.append(dict(episode=e['episode'],clip=e['clip'],goal_hash_errors=errors,inputs=checks['inputs'],previous_action_semantics_mismatch_fraction=checks['previous_action_differs_from_training_semantics']/checks['inputs'],yaw_error_abs_deg=e['yaw_error_abs_deg'],joints=metrics))
  if e['episode']==2:
   for ax,j in zip(axes,[17,21,7]):
    t=[x['t'] for x in series];ax.plot(t,reference[:,j],label='Reference pose',color='#777777');ax.plot(t,target[:,j],label='Policy target (mechanical range applied)',color='#db8850');ax.plot(t,actual[:,j],label='Measured position',color='#138f99');ax.set_title(names[j]);ax.set_ylabel('rad');ax.grid(alpha=.2)
 axes[0].legend(fontsize=9);axes[-1].set_xlabel('Seconds after robot play request');fig.suptitle('Unified policy | dance1_subject2 | episode 2 | actual Q1 data');fig.tight_layout();fig.savefig(a.plot,dpi=150);plt.close(fig)
 (a.root/'reference-and-model-assessment.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n')
 print(json.dumps([{k:v for k,v in x.items() if k!='joints'} for x in out],ensure_ascii=False))
 for i in range(1,4):
  r=json.load(open(a.root/('episode-'+str(i)+'.json')));print(json.dumps({'episode':i,'hal_saturation':r['hal_any_saturation_fraction'],'input_age_ms':r['observation_to_hal_ms'],'arm_mean_tracking_error_rad':sum(x['policy_to_measured_error_rad']['mean'] for x in r['joints'][14:])/8,'hip_roll_tracking_error_rad':[r['joints'][j]['policy_to_measured_error_rad']['mean'] for j in [1,7]]}))


if __name__=='__main__':main()
