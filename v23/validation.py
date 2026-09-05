"""Frozen v21 measured preset vs the compiled OpenCR controller core."""
import argparse
import csv
import hashlib
from functools import lru_cache
import json
from pathlib import Path
import sys
import time

import numpy as np
from core import Core, HERE

ROOT = HERE.parent
sys.path.insert(0, str(ROOT/'v19_bringup'))
sys.path.insert(0, str(ROOT/'v21_pres'))
from tests_v21 import make_engine

def config_for(e):
    w = e.g['mdl']['fwe']['w']
    return dict(p2r=e.g['p2r'], wphi=float(w[0]), wphid=float(w[2]), wbetad=float(w[3]),
                gain=9.5, trigger=.6, relax=.3, vmax=420., amax=8000., tau=.030,
                rest=float(e.p['T_REST']), limit=float(e.p['DELTA_MAX_DEG']))

def source_hashes():
    files = ['v21_pres/sim_engine.py','v21_pres/tests_v21.py','v21_pres/gains_v19sim.py',
             'v19_bringup/params_v19.py','v19_bringup/model_v19.py',
             'v23/firmware/v23_controller/control_core.h','v23/firmware/v23_controller/v23_controller.ino',
             'v23/validation.py']
    return {p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in files}

@lru_cache(maxsize=1)
def measured_plane():
    return make_engine().build_info()['plane']

def simulate(seconds=12, beta=.5, phi=0., seed=0, drive='native', gain=9.5, noise=0.,
             release_ms=0., arm_ms=0., single=False, warmup=False):
    """Same sensor samples feed v21 and native. Native torque drives the unchanged plant.

    release_ms is physical hand release relative to cue; arm_ms enables control.
    Held trials start at -500 ms so both estimators have a measured history.
    """
    e = make_engine()
    e.real['enc_noise_ank_deg'] = noise
    e.real['fwe_async_gamma'] = gain/.95
    e.x0 = dict(phi0=phi, alpha0=beta, theta0=beta)
    e.reset(); e.rng = np.random.default_rng(seed)
    config = config_for(e); config['gain']=gain
    core = Core(config, single=single)
    reference = e.controller
    rows=[]; events=[]; first=None; maxima=dict(Ahat=0.,hold=0.,ref=0.,vref=0.)
    origin = -.5 if warmup or release_ms != 0 or arm_ms != 0 else 0.
    clock=[origin]
    def controller(xe):
        nonlocal first
        x=e.get_state(); f,a,t=np.rad2deg(xe[:3]); d=t-a
        enabled=clock[0]*1000 >= arm_ms and not e.fallen
        # Both paths remain dormant while held/preparing; estimators still run.
        e.fwe['armed']=enabled
        if enabled:
            ref_tau=reference(xe)
        else:
            ref_tau=e.real['pos_kp']*(e.fwe.get('ref',0)-e.hip_local[0])-e.real['pos_kd']*e.hip_local[1]
        out=core.tick(f,a+f,d,*np.rad2deg(e.hip_local),enabled=enabled)
        expected={'Ahat': np.rad2deg(e.risk_A(xe)), 'hold':np.rad2deg(e.fwe['hold']),
                  'ref':np.rad2deg(e.fwe.get('ref',0)), 'vref':np.rad2deg(e.fwe.get('vref',0))}
        errors={k:abs(out[k]-v) for k,v in expected.items()}
        for k,v in errors.items(): maxima[k]=max(maxima[k],v)
        ref_phase={'idle':0,'fold':1,'rest':4}[e.fwe['phase']]
        bad=any(v>1e-7 for v in errors.values()) or out['phase']!=ref_phase
        if bad and first is None: first=len(rows)
        true=np.rad2deg(x[:3]); bp=out['beta']; fp=f
        pred=np.deg2rad([f,bp])+np.array(e.build_info()['plane']['P']) @ np.deg2rad([out['dphi'],out['dbeta']])
        row=dict(seq=len(rows),t_ms=round(clock[0]*1000,3),phi=f,ank=a+f,del_now=d,
                 **out,reference_Ahat=float(expected['Ahat']),reference_hold=float(expected['hold']),
                 reference_ref=float(expected['ref']),reference_phase=e.fwe['phase'],
                 error_Ahat=errors['Ahat'],error_hold=errors['hold'],mismatch=int(bad),
                 beta_pred=float(np.rad2deg(pred[1])),phi_pred=float(np.rad2deg(pred[0])),
                 true_phi=float(true[0]),true_alpha=float(true[1]),true_theta=float(true[2]),
                 enabled=int(enabled),held=int(clock[0]*1000 < release_ms),err=0)
        rows.append(row)
        if out['event']: events.append(dict(seq=row['seq'],t_ms=row['t_ms'],event=int(out['event'])))
        if e.fallen: return 0.
        return ref_tau if drive=='reference' else (e.real['pos_kp']*(out['ref']*np.pi/180-e.hip_local[0])
                    +e.real['pos_kd']*(out['vref']*np.pi/180-e.hip_local[1]))
    e.controller=controller
    # build_info is constant; avoid recomputing model metadata per sample.
    info=e.build_info(); e.build_info=lambda:info
    for n in range(int((seconds-origin)/.005)):
        clock[0]=origin+n*.005
        if clock[0]*1000 < release_ms:
            # Hand fixes the global pose; the hip can still move against that constraint.
            dl=np.rad2deg(e.get_state()[2]-e.get_state()[1])
            e.set_pose(phi,beta-config['p2r']*dl,beta+(1-config['p2r'])*dl)
        e.control_step()
        e.hist.clear()
        if e.fallen: break
    core.close()
    return dict(schema=1,name='v21-native',kind='simulation',config=core.config,
                plane=info['plane'],geom={k:float(e.p[k]) for k in ('R','L1','L2')},
                inputs=dict(seconds=seconds,beta=beta,phi=phi,seed=seed,gain=gain,noise=noise,
                            release_ms=release_ms,arm_ms=arm_ms,drive=drive,single=single,warmup=warmup),
                hashes=source_hashes(), rows=rows,events=events,
                summary=dict(samples=len(rows),fallen=e.fallen,first_mismatch=first,max_errors=maxima,
                             folds=int(rows[-1]['folds']),timeouts=sum(r['event']==3 for r in rows),
                             duration_s=round(rows[-1]['t_ms']/1000,3),
                             phi_rms=float(np.sqrt(np.mean([r['phi']**2 for r in rows])))))

def save(result, stem):
    stem=Path(stem); stem.parent.mkdir(parents=True,exist_ok=True)
    stem.with_suffix('.json').write_text(json.dumps(result,ensure_ascii=False,allow_nan=False),encoding='utf-8')
    with stem.with_suffix('.csv').open('w',encoding='utf-8',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(result['rows'][0]));w.writeheader();w.writerows(result['rows'])

def suite():
    start=time.time(); results=[]
    for beta,phi,noise in [(.5,0,0),(-.5,0,0),(1,-1.64,0),(.5,0,.05)]:
        r=simulate(12,beta,phi,noise=noise)
        assert r['summary']['first_mismatch'] is None, r['summary']
        assert not r['summary']['fallen'], r['summary']
        results.append(dict(inputs=r['inputs'],**r['summary']))
        print('parity',beta,phi,noise,r['summary'],flush=True)
        if beta==.5 and noise==0: save(r,HERE/'reports/baseline')
    # Wide gain plateau, using the native controller on the full plant.
    for gain in [3.8,7.6,11.4]:
        r=simulate(12,gain=gain)
        results.append(dict(inputs=r['inputs'],**r['summary']))
        assert r['summary']['first_mismatch'] is None
        print('gain',gain,r['summary'],flush=True)
    c=Core(); assert c.lib.gate_check()==0; c.close()
    summary=dict(passed=True,seconds=round(time.time()-start,2),cases=results,hashes=source_hashes())
    (HERE/'reports').mkdir(exist_ok=True)
    (HERE/'reports/validation.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    return summary

def startup_suite():
    results=[]
    for release_ms in [-100,0,100,200]:
        r=simulate(12,release_ms=release_ms,warmup=True)
        save(r,HERE/f'reports/start_{release_ms}')
        results.append(dict(release_ms=release_ms,**r['summary']))
    result=dict(cases=results,note='Imposed cue timing and constrained hand pose, not a human/contact model. Gate tested separately.')
    (HERE/'reports/startup.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    return result

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--suite',action='store_true');ap.add_argument('--startup',action='store_true');ap.add_argument('--seconds',type=float,default=12)
    a=ap.parse_args()
    if a.suite: print(json.dumps(suite(),indent=2))
    elif a.startup: print(json.dumps(startup_suite(),indent=2))
    else:
        r=simulate(a.seconds);save(r,HERE/'reports/baseline');print(r['summary'])
