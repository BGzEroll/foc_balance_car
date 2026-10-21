function show_ekf_comparison
here=fileparts(mfilename('fullpath'));addpath(fileparts(here));balance_setup;
out=fullfile(here,'ekf_validation');s=jsondecode(fileread(fullfile(out,'summary.json')));
assert(strcmp(s.status,'complete'),'EKF comparison has not completed');
openfig(fullfile(out,'comparison.fig'),'visible');disp(struct2table(s.runs));
f=replay_measured_course(s.runs(end).result_folder);
f.Name='EKF姿态估计：前进转弯机械真值回放';f.Visible='on';figure(f);
end
