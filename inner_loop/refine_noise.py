"""Refine recorded seeds against nominal, 180 kt and assumed 50 ms plants."""
import json
from pathlib import Path
import numpy as np
from scipy.optimize import minimize
from scipy.io import loadmat
from scipy.signal import ss2tf
root=Path(__file__).resolve().parent
import search_noise as b
n0,d0=b.PN.copy(),b.PD.copy()
old=json.loads((root/'noise_flexible.json').read_text())
r=loadmat(root/'validation.mat',simplify_cells=True)['results']['Models'][0]
n180,d180=ss2tf(r['A'],r['B'].reshape(4,1),np.array([[0,0,1,0]]),np.zeros((1,1)))
n180=-np.trim_zeros(n180[0][:-1],'f')
plants=[(n0,d0),(n180,d180),(n0,np.convolve(d0,[.05,1]))]

def controller(v,family):
    a=np.exp(v);wc=a[0];nz=3 if family==0 else 4;z=a[1:1+nz];p=a[1+nz:]
    nf=np.poly(-z);df=np.poly(-p)
    g=abs(np.polyval(d0,1j*wc)*np.polyval(df,1j*wc)/(np.polyval(n0,1j*wc)*np.polyval(nf,1j*wc)))
    return nf*g,df,dict(CrossoverTarget=wc,Gain=g,Zeros=z.tolist(),Poles=p.tolist(),Family=family)

def evaluate(v,family,details=False):
    ms=[]
    for n,d in plants:
        m=b.assess(v,family,True,plant=(n,d),controller_function=controller)
        if m is None:return 1e8 if not details else None
        ms.append(m)
    m,slow,servo=ms
    score=(1000*max(m['SSE_pct']-.65,0)**2+20*max(m['Overshoot_pct']-16,0)**2
      +1e7*max(m['TailError']-.018,0)**2+100*max(58-m['PhaseMargin_deg'],0)**2
      +1000*max(m['L20_dB']+41.5,0)**2+1000*max(m['T20_dB']+41.5,0)**2
      +1000*max(slow['SSE_pct']-.95,0)**2+30*max(slow['Overshoot_pct']-19,0)**2
      +1e7*max(slow['TailError']-.019,0)**2
      +30*max(servo['Overshoot_pct']-19,0)**2+1e7*max(servo['TailError']-.019,0)**2
      +.1*m['Settling_s']+.01*m['Overshoot_pct'])
    return dict(Score=score,Conditions=ms,LogParameters=v.tolist()) if details else score

rng=np.random.default_rng(1978);results=[]
for family in [0,1]:
    seed=old[family]
    bounds=np.log([[2.02,4.5]]+([[.015,.3],[.1,4],[.5,15]] if family==0 else [[.01,.3],[.02,.6],[.3,5],[1,25]])+
        ([[.001,.3],[.03,5],[2,30],[2,30],[2,30]] if family==0 else [[.001,.3],[.01,2],[.1,10],[2,35],[2,35],[2,35]]))
    seedv=np.log([max(seed['CrossoverTarget'],2.03)]+seed['Zeros']+seed['Poles'])
    for j in range(7):
        start=seedv.copy() if j==0 else np.clip(seedv+rng.normal(0,.25,len(seedv)),bounds[:,0],bounds[:,1])
        opt=minimize(lambda v:evaluate(v,family),start,method='Nelder-Mead',bounds=bounds,
            options=dict(maxiter=2300,xatol=1e-6,fatol=1e-6))
        result=evaluate(opt.x,family,True)
        results.append(result);results.sort(key=lambda x:x['Score'])
        print(f"family={family} start={j} score={result['Score']:.5g}",flush=True)
        if result is results[0]:print(json.dumps(result),flush=True)
        (root/'noise_joint.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
