"""Search real lead/lag compensators with a -40 dB noise constraint."""
import json
from pathlib import Path
import numpy as np
from scipy.optimize import differential_evolution, minimize

HERE = Path(__file__).resolve().parent
AIRCRAFT = json.loads((HERE.parent/'Gqde'/'Gqde.json').read_text(encoding='utf-8'))
PN = -np.array(AIRCRAFT['Numerator'][1:-1], dtype=float)
PD = np.array(AIRCRAFT['Denominator'], dtype=float)
TIMES = np.unique(np.r_[np.arange(0,6.001,.015), np.arange(6.1,25,.1),
                        np.arange(25,80,.5),np.arange(80,121)])
W = np.logspace(-4,3,900)
J = 1j*W
P_FREQ = np.polyval(PN,J)/np.polyval(PD,J)


def controller(v, filters):
    wc,zlag,lag_ratio,zlead,lead_ratio,pf = np.exp(v)
    plag = zlag/lag_ratio
    plead = zlead*lead_ratio
    nf = np.poly([-zlag,-zlead])*pf**filters
    df = np.poly([-plag,-plead]+[-pf]*filters)
    gain = abs(np.polyval(PD,1j*wc)*np.polyval(df,1j*wc)
               /(np.polyval(PN,1j*wc)*np.polyval(nf,1j*wc)))
    return gain*nf,df,dict(CrossoverTarget=wc,Gain=gain,Zlag=zlag,Plag=plag,
                           Zlead=zlead,Plead=plead,Filter=pf,FilterCount=filters)


def assess(v, filters, details=False, plant=None, controller_function=None):
    pn,pd = (PN,PD) if plant is None else plant
    pfreq = P_FREQ if plant is None else np.polyval(pn,J)/np.polyval(pd,J)
    nf,df,par = (controller if controller_function is None else controller_function)(v,filters)
    nl=np.convolve(pn,nf);dl=np.convolve(pd,df)
    cl=dl+np.pad(nl,(len(dl)-len(nl),0))
    poles=np.roots(cl)
    if np.max(poles.real)>=-1e-8:
        return (1e7+1e4*max(0,np.max(poles.real))) if not details else None
    dc=nl[-1]/cl[-1];sse=100*abs(1-dc)
    l20=np.polyval(nl,20j)/np.polyval(dl,20j)
    n20=20*np.log10(abs(l20));t20=20*np.log10(abs(l20/(1+l20)))
    # Simple closed-loop poles give the exact continuous step via residues.
    residues=np.polyval(nl,poles)/(poles*np.polyval(np.polyder(cl),poles))
    y=dc+np.sum(residues[:,None]*np.exp(poles[:,None]*TIMES),axis=0).real
    os=100*max(0,np.max(y)/dc-1)
    outside=np.flatnonzero((abs(y-dc)>.02*abs(dc))|(abs(y-1)>.02))
    ts=0 if not len(outside) else (TIMES[outside[-1]+1] if outside[-1]+1<len(TIMES) else 120)
    tail=TIMES>=2.8
    tail_error=max(np.max(abs(y[tail]-dc)/abs(dc)),np.max(abs(y[tail]-1)))
    wc=par['CrossoverTarget']
    lf=pfreq*np.polyval(nf,J)/np.polyval(df,J)
    tf=lf/(1+lf)
    phase=np.unwrap(np.angle(lf))*180/np.pi
    pm=180+np.interp(np.log(wc),np.log(W),phase)
    ix=np.flatnonzero(abs(tf)<abs(dc)*10**(-3/20))
    bw=W[ix[0]] if len(ix) else 1000
    ms=np.max(abs(1/(1+lf)))
    score=(1000*max(sse-.75,0)**2+30*max(os-17,0)**2
           +300*max(ts-2.85,0)**2+1e5*max(tail_error-.019,0)**2
           +500*max(n20+41,0)**2+500*max(t20+41,0)**2
           +4*max(55-pm,0)**2+100*max(2-bw,0)**2+100*max(bw-10,0)**2
           +100*max(ms-1.8,0)**2+.05*ts+.015*os+.005*(bw-4)**2)
    if not details:
        return float(score)
    return dict(**par,Score=float(score),SSE_pct=sse,Overshoot_pct=os,Settling_s=float(ts),
                PhaseMargin_deg=pm,Bandwidth_radps=bw,L20_dB=n20,T20_dB=t20,
                PeakSensitivity=ms,TailError=tail_error,NumeratorF=nf.tolist(),DenominatorF=df.tolist())


def search():
    bounds=np.log([[2,4.2],[.2,4],[3,120],[.3,7],[1.3,25],[3,22]])
    all_results=[]
    for filters in [2,3,4]:
        result=differential_evolution(lambda x:assess(x,filters),bounds,popsize=13,
             maxiter=190,tol=.002,seed=5560+filters,polish=False,workers=1,
             updating='immediate')
        refined=minimize(lambda x:assess(x,filters),result.x,method='Nelder-Mead',
                         options=dict(maxiter=1800,xatol=2e-5,fatol=1e-5))
        candidate=assess(refined.x,filters,True)
        all_results.append(candidate)
        print(json.dumps(candidate),flush=True)
        (HERE/'noise_search.json').write_text(json.dumps(all_results,indent=2),encoding='utf-8')
    return all_results


if __name__=='__main__':
    search()
