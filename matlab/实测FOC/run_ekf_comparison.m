function summary=run_ekf_comparison
% Resume-safe background comparison. Prior reference physics is reused only
% after checking controller, actuator, sensor, timing and duration snapshots.
here=fileparts(mfilename('fullpath'));addpath(fileparts(here));balance_setup;
out=fullfile(here,'ekf_validation');if ~isfolder(out),mkdir(out);end
diary(fullfile(out,'run.log'));cleanup=onCleanup(@()diary('off'));
summary=struct('status','running','started',char(datetime('now')),'current','unit_checks', ...
 'scope','Six-axis multiplicative error-state EKF; unchanged PID/FOC; simulation only', ...
 'ekf_parameters',measured_ekf_parameters);write_status;
try
 summary.unit_checks=verify_measured_ekf;write_status;
 prior=jsondecode(fileread(fullfile(here,'quaternion_validation','summary.json')));
 assert(strcmp(prior.status,'complete'),'Reference runs missing');
 cases={'recover','route'};durations=[4 10];rows=struct([]);
 for c=1:2
  scenario=cases{c};idx=1;if c==2,idx=4;end
  [cfg,~,p]=measured_pid_config('selected',idx,'explore6',.005);sp=measured_sensor_parameters;
  fc=p.cfg;fc(17:18)=p.B(2);fc(19:20)=p.coulomb(2);
  for m=1:3
   modes={'legacy','quaternion','ekf'};mode=modes{m};summary.current=[scenario '/' mode];write_status;
   tilt=3*double(c==1);
   if m<3
    ix=find(strcmp({prior.runs.scenario},scenario)&strcmp({prior.runs.estimator},mode),1);
    folder=prior.runs(ix).result_folder;
    if isfile(fullfile(folder,'simulation_measured.mat'))
     s=load(fullfile(folder,'simulation_measured.mat'),'o','cfg','fc','sp','p','duration');
     assert(isequal(s.cfg,cfg)&&isequal(s.fc,fc)&&isequal(s.sp.cfg(1:342),sp.cfg(1:342)), 'Reference parameters changed; do not reuse');
     assert(s.duration==durations(c)&&s.p.mechanical_macro_step==250e-6,'Reference timing mismatch');
     o=s.o;r=jsondecode(fileread(fullfile(folder,'validation.json')));r.result_folder=folder;
     reused=true;
    else
     % Published reports survive cleanup; ignored raw MATs may not. Re-run
     % reference physics rather than treating old scalar metrics as fresh data.
     suffix='';if m==2,suffix='_quaternion';end
     folder=fullfile(here,'results',sprintf('%s_selected_explore6_tilt%g_sensor1_macro250us_pid5000us%s_ekf_reference',scenario,tilt,suffix));
     if isfile(fullfile(folder,'validation.json'))&&isfile(fullfile(folder,'simulation_measured.mat'))
      s=load(fullfile(folder,'simulation_measured.mat'),'o','cfg','fc','sp','duration');
      assert(isequal(s.cfg,cfg)&&isequal(s.fc,fc)&&isequal(s.sp.cfg,[sp.cfg m-1])&&s.duration==durations(c),'Resume reference parameters changed');
      o=s.o;r=jsondecode(fileread(fullfile(folder,'validation.json')));r.result_folder=folder;
     else
      [o,r]=run_measured_car(scenario,false,tilt,true,250e-6,durations(c),'selected','explore6',0,.005,mode,'ekf_reference');
     end
     reused=false;
    end
   else
    folder=fullfile(here,'results',sprintf('%s_selected_explore6_tilt%g_sensor1_macro250us_pid5000us_ekf_ekf_comparison',scenario,tilt));
    if isfile(fullfile(folder,'validation.json'))&&isfile(fullfile(folder,'simulation_measured.mat'))
     s=load(fullfile(folder,'simulation_measured.mat'),'o','cfg','fc','sp','duration');
     assert(isequal(s.cfg,cfg)&&isequal(s.fc,fc)&&isequal(s.sp.cfg,[sp.cfg 2])&&s.duration==durations(c),'EKF resume parameters changed');
     o=s.o;r=jsondecode(fileread(fullfile(folder,'validation.json')));r.result_folder=folder;
    else
     [o,r]=run_measured_car(scenario,false,tilt,true,250e-6,durations(c),'selected','explore6',0,.005,'ekf','ekf_comparison');
    end
    reused=false;
   end
   row=ekf_comparison_metrics(o,r,scenario,mode,reused);
   if isempty(rows),rows=row;else,rows(end+1)=row;end
   summary.runs=rows;write_status;
  end
 end
 writetable(struct2table(rows),fullfile(out,'comparison.csv'));
 plot_ekf_comparison(rows,out);
 summary.status='complete';summary.current='';summary.finished=char(datetime('now'));write_status;
 fprintf('EKF_COMPARISON_COMPLETE\n');
catch e
 summary.status='failed';summary.error=getReport(e,'extended','hyperlinks','off');write_status;rethrow(e);
end
 function write_status
  fid=fopen(fullfile(out,'summary.json'),'w');fprintf(fid,'%s',jsonencode(summary,PrettyPrint=true));fclose(fid);
 end
end
