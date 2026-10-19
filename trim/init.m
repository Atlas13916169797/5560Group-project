function [F, X0, U0, root] = init()
% Load only Group 8 data; all paths are relative to this file.
root = fileparts(mfilename('fullpath'));
addpath(root);
F = FlightDataAircraft8();
F.Geometric = F.Geo;
F.Inertial = F.I;
F.Propeller = F.Prop;
D = load(fullfile(root, 'ICsAircraft8_200knts_300ft.mat'), 'X0', 'U0');
X0 = D.X0(:);
U0 = D.U0(:);
assert(numel(X0) == 12 && numel(U0) == 5, 'Unexpected data dimensions.');
assert(all(isfinite([X0; U0])), 'Non-finite initial data.');
end
