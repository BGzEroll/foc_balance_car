try
set(groot,'defaultFigureVisible','off');
cd('D:/code/Embedded-related/balance_car'); here=fullfile(pwd,'matlab','实测FOC'); addpath(fullfile(pwd,'matlab')); balance_setup; addpath(here,'-begin');
verify_pid_rate_math; [cfg,d,p]=measured_pid_config; assert(cfg(31)==.005 && d.encoder_period_s==.005);
model='balance_car_pid_foc'; load_system(fullfile(here,[model '.slx'])); mw=get_param(model,'ModelWorkspace'); assignin(mw,'suite_config',cfg);
sp=measured_sensor_parameters; configure_measured_timing(model,.005,.00025,sp.Ts);
notes=find_system(model,'FindAll','on','Type','annotation');
for k=1:numel(notes)
 txt=get_param(notes(k),'Name'); txt=strrep(txt,'PID ONLY at 1kHz','PID ONLY at 200Hz (default; optional 1kHz)'); txt=strrep(txt,'Encoder observations 1kHz; MPU6050/AR12/complementary filter remain 200Hz (held between frames)','Encoder observations and MPU6050/AR12/complementary filter at 200Hz by default'); set_param(notes(k),'Name',txt);
end
save_system(model); close_system(model,0);
[~,r]=run_measured_car('recover',false,3,true,.0005,.05,'selected','bench');
assert(r.timing.pid_period_s==.005 && r.timing.encoder_period_s==.005 && r.timing_validation_pass); assert(r.pid_tick_count==11);
fid=fopen(fullfile(here,'pid_rate_validation','default_200Hz_restored.json'),'w'); fprintf(fid,'%s',jsonencode(struct('default_pid_period_s',.005,'timing',r.timing,'logged_pid_period_s',r.logged_pid_period_s,'pid_tick_count',r.pid_tick_count,'timing_validation_pass',r.timing_validation_pass,'smoke_duration_s',.05,'scope','Default rate and compiled execution smoke check only; full-course evidence remains in pid_validation/summary.json'),PrettyPrint=true)); fclose(fid);
fprintf('DEFAULT_200HZ_RESTORED_OK\n');
catch err
fprintf(2,'%s\n',getReport(err,'extended')); exit(1);
end
exit(0);