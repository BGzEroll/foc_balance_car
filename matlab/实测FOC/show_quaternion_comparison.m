function show_quaternion_comparison
here=fileparts(mfilename('fullpath'));addpath(fileparts(here));balance_setup;
out=fullfile(here,'quaternion_validation');s=jsondecode(fileread(fullfile(out,'summary.json')));
assert(strcmp(s.status,'complete'),'Comparison has not completed');
s=refresh_quaternion_metrics;
openfig(fullfile(out,'kinematics.fig'),'visible');openfig(fullfile(out,'comparison.fig'),'visible');
disp(struct2table(s.runs));
% Saved native 3D mechanical truth; no new simulation is started.
replay_measured_course(s.runs(end).result_folder);
end
