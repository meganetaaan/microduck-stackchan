"""Run the existing CPU MuJoCo probe with process-lifetime ORT telemetry OFF.

ORT1.30 starts telemetry during import, so opt out BEFORE importing it. This
wrapper leaves the original delivered run_probe.py and all physics unchanged.
Official source: microsoft/onnxruntime v1.30.0 docs/Privacy.md.
"""
import os
os.environ['ORT_DISABLE_TELEMETRY']='1'
import runpy,json
from pathlib import Path
result=runpy.run_path(str(Path(__file__).resolve().parents[1]/'run_probe.py'),run_name='__main__')
out=Path(str(result['args'].out)+'.json')
data=json.loads(out.read_text())
data['runtime_privacy']={'ORT_DISABLE_TELEMETRY':'1','set_before_import':True,'source':'https://raw.githubusercontent.com/microsoft/onnxruntime/v1.30.0/docs/Privacy.md','note':'Official process-lifetime opt-out. No network trace claim is made.'}
out.write_text(json.dumps(data,indent=2))
