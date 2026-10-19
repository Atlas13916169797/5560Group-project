% Run from any working directory. Requires base MATLAB only.
folder = fileparts(mfilename('fullpath'));
addpath(folder);
linear_result = calc_linear();
