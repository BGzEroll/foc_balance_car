function verify_pipeline
% Synthetic workflow test only. Does not claim a fitted model of the actual car.
here=fileparts(mfilename('fullpath'));root=fileparts(fileparts(here));
prep=fullfile(root,'esp32','tuning_preparation');folder=fullfile(prep,'results','synthetic_pipeline');
if ~isfolder(folder),mkdir(folder);end
schema=jsondecode(fileread(fullfile(prep,'telemetry_schema.json')));
profile=jsondecode(fileread(fullfile(prep,'profiles','wired_B_current.json')));
n=2400;t=(0:n-1)'*.005;
T=array2table(zeros(n,numel(schema.fields)),'VariableNames',cellstr(schema.fields));
u=.3*sin(2*t)+.12*sin(5.7*t)+.05*sin(11.1*t);
A=[0 1 0;-15 -2 .3;.5 .2 -1];B=[0;2;1];x=zeros(3,n);
for k=2:n,x(:,k)=expm(A*.005)*x(:,k-1)+(A\((expm(A*.005)-eye(3))*B))*u(k-1);end
T.sample_us=1e6+t*1e6;T.imu_us=T.sample_us-500;T.cycle=(1:n)';T.dt_us(:)=5000;
T.mode(:)=2;T.flags(:)=7;T.revision(:)=1;T.current_limit_A(:)=.7;
T.pitch=x(1,:)'+profile.gains.pitch_trim_rad;T.pitch_rate=x(2,:)';T.speed=x(3,:)';
T.iq_a=-u/2;T.iq_b=u/2;T.command_a=u/2;T.command_b=u/2;
names=fieldnames(profile.gains);
for k=1:numel(names),T.(names{k})(:)=profile.gains.(names{k});end
writetable(T,fullfile(folder,'samples.csv'));
fid=fopen(fullfile(folder,'metadata.json'),'w');fprintf(fid,'%s',jsonencode(struct('source','SYNTHETIC PIPELINE TEST; not actual hardware')));fclose(fid);
model=fit_current_plant(folder,[1 7]);assert(model.valid_for_search);
blocked=false;
try,explore_current_pid(fullfile(folder,'identified_plant.mat'));catch e,blocked=contains(e.message,'Multi-step validation');end
assert(blocked,'Single-step fit must not authorize gain search');
validation=validate_disturbance_model(folder,[1 7],[7.5 11.5]);
assert(validation.accepted_for_disturbance_search,'Synthetic multi-step validation failed');
explore_current_pid(fullfile(folder,'identified_plant.mat'));
% Insufficient/idle data must not silently produce a plant model.
T.mode(:)=0;idle=fullfile(folder,'idle_reject');if ~isfolder(idle),mkdir(idle);end
writetable(T,fullfile(idle,'samples.csv'));
copyfile(fullfile(folder,'metadata.json'),fullfile(idle,'metadata.json'));
rejected=false;try,fit_current_plant(idle,[1 11]);catch e,rejected=contains(e.message,'600 valid active cycles');end
assert(rejected,'Idle-data identification gate failed');
disp('SYNTHETIC_PIPELINE_AND_IDLE_GATE_OK');
end
