function show_encoder_yaw_comparison
here=fileparts(mfilename('fullpath'));addpath(fileparts(here));balance_setup;
out=fullfile(here,'encoder_yaw_validation');s=jsondecode(fileread(fullfile(out,'summary.json')));
assert(strcmp(s.status,'complete'),'A/B/C comparison has not completed');
diagnose_encoder_yaw_gate;
openfig(fullfile(out,'offline.fig'),'visible');openfig(fullfile(out,'comparison.fig'),'visible');disp(struct2table(s.runs));
f=replay_measured_course(s.runs(end).result_folder);f.Name='C：编码器确认未转向 + 零角速度KF';f.Visible='on';figure(f);drawnow;
end
