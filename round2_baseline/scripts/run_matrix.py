#!/usr/bin/env python3
"""Sequential development runs; preserves every result including failures."""
import argparse
from pathlib import Path
import subprocess
import sys

p=argparse.ArgumentParser();p.add_argument('prefix');p.add_argument('seeds',nargs='+',type=int);args=p.parse_args()
scripts=Path(__file__).resolve().parent
for seed in args.seeds:
    folder=Path(args.prefix+'_s'+str(seed))
    print('START SEED',seed,flush=True)
    code=subprocess.call([sys.executable,str(scripts/'run_trial.py'),str(folder),'--cones','--seed',str(seed),'--timer','--low-com','--audit-cones'])
    if (folder/'poses.json').exists():
        with (folder/'analysis_stdout.txt').open('w') as out:
            subprocess.call([sys.executable,str(scripts/'analyze_trial.py'),str(folder)],stdout=out)
    print('FINISHED SEED',seed,'exit',code,flush=True)
