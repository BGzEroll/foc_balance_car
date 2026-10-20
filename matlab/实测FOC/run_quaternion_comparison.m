function summary=run_quaternion_comparison
% Background entry point. Keep the original controller/electrical parameters.
here=fileparts(mfilename('fullpath'));addpath(fileparts(here));balance_setup;
out=fullfile(here,'quaternion_validation');if ~isfolder(out),mkdir(out);end
diary(fullfile(out,'run.log'));cleanup=onCleanup(@()diary('off'));
summary=struct('status','running','started',char(datetime('now')),'scope', ...
 'Six-axis attitude estimation comparison; unchanged PID and FOC; simulation only');
write_status;
try
 summary.kinematics=verify_quaternion_kinematics(out);write_status;
 cases={'recover','route'};durations=[4 10];modes={'legacy','quaternion'};rows=struct([]);
 for c=1:numel(cases)
  for m=1:2
   summary.current=[cases{c} '/' modes{m}];write_status;
   tilt=0;if c==1,tilt=3;end
   [o,r]=run_measured_car(cases{c},false,tilt,true,250e-6,durations(c),'selected','explore6',0,.005,modes{m},'attitude_comparison');
   row=metrics(o,r,cases{c},modes{m});
   if isempty(rows),rows=row;else,rows(end+1)=row;end
   summary.runs=rows;write_status;
  end
 end
 writetable(struct2table(rows),fullfile(out,'comparison.csv'));
 f=figure('Visible','off','Color','w','Position',[50 50 1400 850]);tl=tiledlayout(2,2,'Padding','compact');
 for c=1:2
  nexttile;hold on;
  for m=1:2
   s=load(fullfile(rows((c-1)*2+m).result_folder,'simulation_measured.mat'),'o');
   [t,y]=values(s.o.control3d);[tr,raw]=values(s.o.raw3d);[pitch,yaw]=attitude(raw);
   pt=interp1(tr,pitch,t);plot(t,rad2deg(y(:,5)+deg2rad(-.0896225217620993)-pt),'DisplayName',modes{m});
  end
  grid on;legend('Location','best');xlabel('时间 / s');ylabel('估计俯仰 - 真值 / °');title(cases{c});
  nexttile;hold on;
  for m=1:2
   s=load(fullfile(rows((c-1)*2+m).result_folder,'simulation_measured.mat'),'o');
   [t,y]=values(s.o.control3d);[tr,raw]=values(s.o.raw3d);[~,yaw]=attitude(raw);
   plot(tr,rad2deg(unwrap(yaw)),'DisplayName',[modes{m} ' 真值']);
   if m==1,plot(t,rad2deg(y(:,13)),'k--','DisplayName','目标航向');end
  end
  grid on;legend('Location','best');xlabel('时间 / s');ylabel('真实偏航 / °');title(cases{c});
 end
 title(tl,'姿态算法闭环对照：相同PID / IMU 200Hz / FOC / 250µs机械步长');
 exportgraphics(f,fullfile(out,'comparison.png'),'Resolution',150);savefig(f,fullfile(out,'comparison.fig'));close(f);
 summary.status='complete';summary.finished=char(datetime('now'));summary.current='';write_status;
 fprintf('QUATERNION_COMPARISON_COMPLETE\n');
catch e
 summary.status='failed';summary.error=getReport(e,'extended','hyperlinks','off');write_status;rethrow(e);
end
 function write_status
  fid=fopen(fullfile(out,'summary.json'),'w');fprintf(fid,'%s',jsonencode(summary,PrettyPrint=true));fclose(fid);
 end
end
function row=metrics(o,r,scenario,mode)
[t,y]=values(o.control3d);[tr,raw]=values(o.raw3d);[pitch,yaw]=attitude(raw);
keep=t>=tr(1)&t<=tr(end);t=t(keep);y=y(keep,:);
pe=y(:,5)+deg2rad(-.0896225217620993)-interp1(tr,pitch,t);
ye=atan2(sin(y(:,7)-interp1(tr,unwrap(yaw),t)),cos(y(:,7)-interp1(tr,unwrap(yaw),t)));
row=struct('scenario',scenario,'estimator',mode,'completed',r.completed,'fell',r.fell, ...
 'pitch_estimate_rms_deg',rad2deg(sqrt(mean(pe.^2))), ...
 'yaw_estimate_rms_deg',rad2deg(sqrt(mean(ye.^2))), ...
 'true_peak_tilt_deg',r.max_true_tilt_error_deg,'true_final_yaw_error_deg',r.true_final_yaw_error_deg, ...
 'peak_iq_A',r.peak_actual_each_iq_A,'result_folder',r.result_folder);
end
function [p,y]=attitude(raw)
a=balance_control_coordinates(raw);q=a(:,8:11);
r11=1-2*(q(:,3).^2+q(:,4).^2);r21=2*(q(:,2).*q(:,3)+q(:,1).*q(:,4));r31=2*(q(:,2).*q(:,4)-q(:,1).*q(:,3));
p=atan2(-r31,hypot(r11,r21));y=atan2(r21,r11);
end
function [t,v]=values(ts)
t=ts.Time;v=squeeze(ts.Data);if size(v,1)~=numel(t),v=v';end
[t,i]=unique(t,'stable');v=v(i,:);
end
