function report=validate_disturbance_model(folder,train_window,test_window)
% Fit on natural motion; validate on a separate operator-reported push segment.
% The current input remains measured Iq; this is plant replay, not a closed-loop forecast.
assert(train_window(2)<test_window(1)||test_window(2)<train_window(1),'Training and validation must not overlap');
try
    model=fit_current_plant(folder,train_window);
catch err
    if ~contains(err.message,'Holdout error too large')||~isfile(fullfile(folder,'identified_plant.mat')),rethrow(err);end
    data=load(fullfile(folder,'identified_plant.mat'),'model');model=data.model;
end
[T,~]=analyze_session(folder);
valid=T.time_s>=test_window(1)&T.time_s<=test_window(2)&T.mode==2&...
    bitand(uint32(T.flags),uint32(7))==7&T.fault==0&...
    T.imu_age_us<=20000&T.encoder_age_us<=1500&...
    T.current_age_a_us<1000&T.current_age_b_us<1000;
T=T(valid,:);assert(height(T)>=600&&max(diff(T.cycle))==1,'Need contiguous active validation');
x=[T.pitch-T.pitch_trim_rad,T.pitch_rate,T.speed];u=-T.iq_a+T.iq_b;
h=.005;AB=expm([model.A model.B model.bias;zeros(2,5)]*h);
A=AB(1:3,1:3);B=AB(1:3,4);bias=AB(1:3,5);
rows=[];
for horizon=[.05 .1 .25 .5 1]
    count=round(horizon/h);errors=[];persistence=[];
    for start=1:20:height(T)-count
        z=x(start,:)';
        for j=start:start+count-1,z=A*z+B*u(j)+bias;end
        errors(end+1,:)=z'-x(start+count,:); %#ok<AGROW>
        persistence(end+1,:)=x(start,:)-x(start+count,:); %#ok<AGROW>
    end
    rows(end+1,:)=[horizon,sqrt(mean(errors.^2)),sqrt(mean(persistence.^2))]; %#ok<AGROW>
end
% One-step derivative fit is insufficient: require useful multi-step predictions.
accepted=model.valid_for_search&&all(rows(2:4,2:4)<rows(2:4,5:7),'all');
model.one_step_valid_for_search=model.valid_for_search;
model.valid_for_search=accepted;
model.disturbance_validation_required=true;
model.disturbance_validation_passed=accepted;
save(fullfile(folder,'identified_plant.mat'),'model');
fid=fopen(fullfile(folder,'identified_plant.json'),'w');fprintf(fid,'%s',jsonencode(model));fclose(fid);
report=struct('train_window_s',train_window,'test_window_s',test_window,...
    'columns',{{'horizon_s','pitch_RMSE_rad','rate_RMSE_rad_s','speed_RMSE_m_s',...
    'persistence_pitch','persistence_rate','persistence_speed'}},'rows',rows,...
    'accepted_for_disturbance_search',accepted,...
    'scope','Separate-time measured-current replay. Operator push time approximate; closed-loop identification can be biased. No physical mass/inertia changes.');
fid=fopen(fullfile(folder,'disturbance_model_validation.json'),'w');fprintf(fid,'%s',jsonencode(report));fclose(fid);
disp(report);disp(array2table(rows,'VariableNames',report.columns));
end
