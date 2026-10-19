function model = loop(Kq)
% Construct the pitch-rate loop with a supplied, signed compensator.
% Ideal elevator actuator Ae=1, ideal rate sensor Hq=1.
% All signals are trim increments: q in rad/s; elevator in rad.
if nargin ~= 1
    error('Group8:ControllerRequired', 'Supply the signed compensator Kq.');
end
folder = fileparts(mfilename('fullpath'));
D = load(fullfile(fileparts(folder),'Gqde','Gqde.mat'),'Gqde');
Gqde = D.Gqde;
Lq = Gqde*Kq;
model = struct('Gqde',Gqde,'Kq',Kq,'Lq',Lq, ...
    'Sq',feedback(1,Lq),'Cq',feedback(Lq,1),'Uq',feedback(Kq,Gqde));
% Cq = q/qc; Sq = error/qc; Uq = elevator increment/qc.
% This function does not select or tune Kq or imply closed-loop stability.
end
