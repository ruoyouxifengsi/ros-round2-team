import json,sys
from pathlib import Path
for name in sys.argv[1:]:
    folder=Path(__file__).parent/name
    t=json.loads((folder/'trace.json').read_text());begins={}
    rows=[]
    for p in t['phases']:
        if p['edge']=='begin':begins[p['id']]=p;continue
        b=begins.get(p['id'])
        if b:rows.append(dict(phase=p['phase'],start=b['stamp'],sim=round(p['stamp']-b['stamp'],3),wall=round(p['wall']-b['wall'],3)))
    (folder/'phases.json').write_text(json.dumps(rows,indent=2))
    print(name,rows)
