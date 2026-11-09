"""Replay private real Q1 observations through exact JIT/ONNX. No ROS or commands."""
import argparse,hashlib,json,time
from pathlib import Path
import numpy as np
import torch
import onnxruntime as ort


def sha(p):
 with p.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()


def main():
 p=argparse.ArgumentParser();p.add_argument('--observations',type=Path,required=True);p.add_argument('--jit',type=Path,required=True);p.add_argument('--model-dir',type=Path,required=True);p.add_argument('--report',type=Path,required=True);a=p.parse_args()
 data=json.load(open(a.observations));assert data['source']=='private_real_Q1_actor_input_logs';records=data['records'];assert records
 release=json.load(open(a.model_dir/'release.json'));assert sha(a.jit)==release['source_jit_sha256'];assert sha(a.model_dir/'policy.onnx')==release['onnx_sha256']
 torch.set_num_threads(2);torch.set_num_interop_threads(1);jit=torch.jit.load(str(a.jit),map_location='cpu').eval()
 options=ort.SessionOptions();options.intra_op_num_threads=2;options.inter_op_num_threads=1;options.execution_mode=ort.ExecutionMode.ORT_SEQUENTIAL
 session=ort.InferenceSession(str(a.model_dir/'policy.onnx'),sess_options=options,providers=['CPUExecutionProvider'])
 maxima={'jit_vs_onnx':0.,'onboard_vs_jit':0.,'onboard_vs_onnx':0.};squared=0.;terms=0;modes={};onboard_count=0;failures=[];worst=None;began=time.monotonic()
 with torch.inference_mode():
  for i,row in enumerate(records):
   x=np.asarray(row['input'],dtype=np.float32).reshape(1,1376);assert np.isfinite(x).all();mode=str(row['context']['mode']);modes[mode]=modes.get(mode,0)+1
   j=jit(torch.from_numpy(x.copy())).cpu().numpy();o=session.run(['actions'],{'obs':x})[0];delta=np.abs(j-o);error=float(delta.max());squared+=float(np.square(delta.astype(np.float64)).sum());terms+=delta.size
   if error>maxima['jit_vs_onnx']:worst={'id':row['id'],'epoch':row['epoch'],'context':row['context'],'max_error':error}
   maxima['jit_vs_onnx']=max(maxima['jit_vs_onnx'],error)
   if not np.allclose(j,o,rtol=3e-4,atol=3e-5):failures.append({'id':row['id'],'comparison':'jit_vs_onnx'})
   if row['onboard_raw'] is not None:
    b=np.asarray(row['onboard_raw'],dtype=np.float32).reshape(1,22);onboard_count+=1
    for name,other in [('onboard_vs_jit',j),('onboard_vs_onnx',o)]:
     maxima[name]=max(maxima[name],float(np.abs(b-other).max()))
     if not np.allclose(b,other,rtol=3e-4,atol=3e-5):failures.append({'id':row['id'],'comparison':name})
   if (i+1)%500==0:print(json.dumps({'checked':i+1,'max_abs_error':maxima}),flush=True)
 report={'source':data['source'],'inputs_sha256':sha(a.observations),'observation_history':'as actually recorded, not synthesized or counterfactually patched','samples':len(records),'onboard_outputs_compared':onboard_count,'modes':modes,'jit_sha256':sha(a.jit),'onnx_sha256':sha(a.model_dir/'policy.onnx'),'versions':{'torch':torch.__version__,'onnxruntime':ort.__version__,'numpy':np.__version__},'max_abs_errors':maxima,'jit_onnx_rmse':(squared/terms)**.5,'worst':worst,'rtol':3e-4,'atol':3e-5,'failures':failures,'success':not failures,'hardware_command_sent':False,'elapsed_s':time.monotonic()-began}
 a.report.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');a.report.chmod(0o600);print(json.dumps(report,ensure_ascii=False));assert not failures


if __name__=='__main__':main()
