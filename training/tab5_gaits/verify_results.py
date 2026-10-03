"""Verify saved outputs without executing the policy on hardware."""
import os
os.environ['ORT_DISABLE_TELEMETRY']='1'
os.environ['OPENBLAS_NUM_THREADS']='1'
from pathlib import Path
import argparse,json,hashlib
import torch,numpy as np,onnxruntime as ort,imageio.v2 as imageio
from env import ROOT
from policy import Actor
from calibrate_commands import CalibratedActor
p=argparse.ArgumentParser();p.add_argument('--folder',type=Path,default=Path(__file__).resolve().parent);a=p.parse_args();folder=a.folder;torch.set_num_threads(1)
ck=torch.load(folder/'results/policy.pt',weights_only=False);core=Actor();core.load_state_dict(ck['actor']);actor=CalibratedActor(core);actor.calibration.copy_(ck['command_calibration']);opts=ort.SessionOptions();opts.intra_op_num_threads=1;opts.inter_op_num_threads=1;s=ort.InferenceSession(str(folder/'results/policy.onnx'),sess_options=opts,providers=['CPUExecutionProvider']);x=np.random.default_rng(919).normal(0,.1,(100,39)).astype(np.float32);x[:,5]=-1
with torch.no_grad():expected=actor(torch.from_numpy(x)).numpy()
actual=s.run(None,{'obs':x})[0];error=float(np.max(np.abs(actual-expected)));assert error<2e-6
manifest=json.loads((folder/'results/training/frozen_model_manifest.json').read_text());verified=0
for key,want in manifest['files'].items():
 path=Path(key) if Path(key).is_absolute() else ROOT/key
 if not path.exists():path=ROOT/key.split('/microduck-stackchan/',1)[-1]
 assert hashlib.sha256(path.read_bytes()).hexdigest()==want,str(path);verified+=1
v=imageio.get_reader(folder/'results/video/trained_gaits.mp4');meta=v.get_meta_data();frames=v.count_frames();v.close();assert frames==1200
report={'onnx_checkpoint_max_abs_error':error,'onnx_input_shape':['batch',39],'onnx_output_shape':['batch',10],'onnx_parity_samples':100,'frozen_files_verified':verified,'video_frames':frames,'video_fps':meta['fps'],'video_duration_s':meta['duration'],'video_size':list(meta['size']),'policy_sha256':hashlib.sha256((folder/'results/policy.onnx').read_bytes()).hexdigest(),'runtime_privacy':{'ORT_DISABLE_TELEMETRY':'1','set_before_import':True,'network_trace_performed':False}}
(folder/'results/final_verification.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
