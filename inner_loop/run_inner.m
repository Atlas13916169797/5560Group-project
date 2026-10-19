% Reproduce the selected controller, engineering checks and figures.
folder=fileparts(mfilename('fullpath'));addpath(folder);
logFile=fullfile(folder,'inner_log.txt');fid=fopen(logFile,'w');fclose(fid);
diary(logFile);guard=onCleanup(@() diary('off'));
design_inner();validate_inner();export_inner();plot_inner();
fprintf('All inner-loop files saved to %s\n',folder);
diary('off');clear guard;
