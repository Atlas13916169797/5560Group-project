function open_sisotool(iteration)
% Open a saved compensator in SISOtool.
% Pq includes the sign reversal and one integrator.
if nargin<1,iteration=3;end
folder=fileparts(mfilename('fullpath'));D=load(fullfile(folder,'controller.mat'));
assert(ismember(iteration,1:3),'Choose iteration 1, 2 or 3.');
sisotool(D.Pq,D.design.F{iteration});
% Convert exported Fnew to the physical controller with Kq=-Fnew/s.

end
