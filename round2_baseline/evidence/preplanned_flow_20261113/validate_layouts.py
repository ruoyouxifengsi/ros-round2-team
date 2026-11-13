import json,subprocess,sys
from pathlib import Path
here=Path(__file__).resolve().parent
try:
    for seed in (1,'restore',42):
        subprocess.run([sys.executable,str(here/'set_layout.py'),str(seed)],check=True)
        name='final_current' if seed=='restore' else 'final_seed_'+str(seed)
        print('VALIDATING '+name,flush=True)
        with (here/(name+'_runner.log')).open('w') as log:
            subprocess.run([sys.executable,str(here/'compare.py'),name,'--candidate','--full'],stdout=log,stderr=subprocess.STDOUT,check=True)
        subprocess.run([sys.executable,str(here/'assess_full.py'),name],check=True,stdout=subprocess.DEVNULL)
        r=json.loads((here/name/'assessment.json').read_text())
        print(json.dumps(dict(name=name,accepted=r['accepted'],sim_seconds=r['sim_seconds'],checks=r['checks'])),flush=True)
        assert r['accepted'],'Full-route layout validation failed'
finally:
    subprocess.run([sys.executable,str(here/'set_layout.py'),'restore'],check=True)
