function m = inner_metrics(L)
% Pitch-rate tracking metrics.
T=minreal(feedback(L,1),1e-8); S=minreal(feedback(1,L),1e-8);
m=struct('Stable',isstable(T),'DC',NaN,'SSE_pct',Inf,'Bandwidth_radps',NaN,...
 'Rise_s',Inf,'SettlingFinal_s',Inf,'SettlingCommand_s',Inf,...
 'Overshoot_pct',Inf,'GainMargin_dB',NaN,'PhaseMargin_deg',NaN,...
 'Crossover_radps',NaN,'L20_dB',NaN,'T20_dB',NaN,...
 'LNoiseMax_dB',NaN,'TNoiseMax_dB',NaN,'NoisePassed',false,'PeakSensitivity',NaN,...
 'EquivalentNoiseBandwidth_radps',NaN);
if ~m.Stable,return;end
t=(0:.002:120)'; y=squeeze(step(T,t)); dc=dcgain(T);
info=stepinfo(y,t,dc,'SettlingTimeThreshold',.02);
last=find(abs(y-1)>.02,1,'last'); ts=Inf;
if isempty(last),ts=0;elseif last<numel(t),ts=t(last+1);end
[gm,pm,~,wc]=margin(L);
w=logspace(-5,5,14000); h=squeeze(freqresp(T,w));
m.DC=dc; m.SSE_pct=100*abs(1-dc); m.Bandwidth_radps=bandwidth(T);
m.Rise_s=info.RiseTime; m.SettlingFinal_s=info.SettlingTime;
m.SettlingCommand_s=ts; m.Overshoot_pct=info.Overshoot;
m.GainMargin_dB=20*log10(gm); m.PhaseMargin_deg=pm;
m.Crossover_radps=wc; m.T20_dB=20*log10(abs(evalfr(T,20i)));
m.L20_dB=20*log10(abs(evalfr(L,20i)));
noiseW=logspace(log10(20),5,5000);
m.LNoiseMax_dB=20*log10(max(abs(squeeze(freqresp(L,noiseW)))));
m.TNoiseMax_dB=20*log10(max(abs(squeeze(freqresp(T,noiseW)))));
m.NoisePassed=m.LNoiseMax_dB<=-40 && m.TNoiseMax_dB<=-40;
m.PeakSensitivity=norm(S,inf);
% One-sided noise area normalized to DC gain.
m.EquivalentNoiseBandwidth_radps=trapz(w,abs(h).^2)/abs(dc)^2;
end
