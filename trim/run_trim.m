% Run this script from any working directory. Requires base MATLAB only.
project = fileparts(mfilename('fullpath'));
addpath(project);
trim_result = check_trim();
