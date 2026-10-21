function summary=run_encoder_yaw_comparison
here=fileparts(mfilename('fullpath'));addpath(fileparts(here));balance_setup;
out=fullfile(here,'encoder_yaw_validation');if ~isfolder(out),mkdir(out);end
diary(fullfile(out,'run.log'));cleanup=onCleanup(@()diary('off'));
summary=struct('status','running','current','checks','started',char(datetime('now')), ...
 'parameters',encoder_yaw_parameters,'scope','Encoder-gated yaw only; same legacy pitch filter, PID and FOC; simulation only');write_status;
try
 summary.checks=verify_encoder_yaw;write_status;
 summary.current='offline';write_status;summary.offline=simulate_encoder_yaw_offline(out);write_status;
 modes={'yaw_a','yaw_b','yaw_c'};rows=struct([]);
 for m=1:3
  mode=modes{m};summary.current=['native/' mode];write_status;
  folder=fullfile(here,'results',['yaw_test_selected_explore6_tilt0_sensor1_macro250us_pid5000us_' mode '_biasdrift_encoder_yaw']);
  if isfile(fullfile(folder,'simulation_measured.mat'))&&isfile(fullfile(folder,'validation.json'))
   s=load(fullfile(folder,'simulation_measured.mat'),'o','sp','duration','cfg','fc');
   [cfg,~,p]=measured_pid_config('selected',12,'explore6',.005);sp=measured_sensor_parameters;
   fc=p.cfg;fc(17:18)=p.B(2);fc(19:20)=p.coulomb(2);
   assert(isequal(s.cfg,cfg)&&isequal(s.fc,fc)&&isequal(s.sp.cfg,[sp.cfg m+2 1])&&s.duration==14,'Resume settings mismatch');
   o=s.o;r=jsondecode(fileread(fullfile(folder,'validation.json')));r.result_folder=folder;
  else
   [o,r]=run_measured_car('yaw_test',false,0,true,250e-6,14,'selected','explore6',0,.005,mode,'encoder_yaw','bias_drift');
  end
  row=ekf_comparison_metrics(o,r,'yaw_test',mode,false);
  [t,d]=data(o.yaw_observer);[tr,raw]=data(o.raw3d);a=balance_control_coordinates(raw);q=a(:,8:11);
  yaw=unwrap(atan2(2*(q(:,2).*q(:,3)+q(:,1).*q(:,4)),1-2*(q(:,3).^2+q(:,4).^2)));
  keep=t>=tr(1)&t<=tr(end);t=t(keep);d=d(keep,:);trueRate=interp1(tr,gradient(yaw,tr),t);
  held=(d(:,7)>.5)&(m>=2);
  row.confirmed_no_turn_fraction=mean(d(:,7)>.5);row.false_hold_above_1deg_s_s=.005*sum(held&abs(trueRate)>deg2rad(1));
  row.true_rotation_during_hold_deg=rad2deg(trapz(t,abs(trueRate).*held));row.final_estimated_bias_deg_s=rad2deg(d(end,4));
  row.turn_bias_profile='body gyro Z +0.25deg/s +0.015deg/s^2*t; before quantization';
  if isempty(rows),rows=row;else,rows(end+1)=row;end
  summary.runs=rows;write_status;
 end
 writetable(struct2table(rows),fullfile(out,'comparison.csv'));plot_encoder_yaw_comparison(rows,out);
 summary.status='complete';summary.current='';summary.finished=char(datetime('now'));write_status;fprintf('ENCODER_YAW_COMPLETE\n');
catch e
 summary.status='failed';summary.error=getReport(e,'extended','hyperlinks','off');write_status;rethrow(e);
end
 function write_status
  fid=fopen(fullfile(out,'summary.json'),'w');fprintf(fid,'%s',jsonencode(summary,PrettyPrint=true));fclose(fid);
 end
end
function [t,v]=data(ts)
t=ts.Time;v=squeeze(ts.Data);if size(v,1)~=numel(t),v=v';end;[t,i]=unique(t,'stable');v=v(i,:);
end
