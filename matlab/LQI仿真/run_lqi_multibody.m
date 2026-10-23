function summaries=run_lqi_multibody
% 主分支原生Multibody对象的独立LQI副本，未修改原SLX实体。
here=fileparts(mfilename('fullpath'));root=fileparts(fileparts(here));addpath(here,fullfile(root,'matlab'),fullfile(root,'matlab','3D仿真'),fullfile(root,'matlab','LQR参数计算'));
balance_setup;mesh=fullfile(root,'模型','仿真模型','通用URDF','meshes');addpath(mesh);
source=fullfile(root,'matlab','3D仿真','balance_car_3d_suite.slx');h=load_system(source);model=get_param(h,'Name');
chart=find(sfroot,'-isa','Stateflow.EMChart','Path',[model '/Balance_Controller']);chart.Script=fileread(fullfile(root,'matlab','3D仿真','balance_suite_controller.m'));
[cfg,d]=balance_control_config('lqi',1,2,[1 1],'normal');mw=get_param(h,'ModelWorkspace');assignin(mw,'suite_config',cfg);
set_param(model,'SimMechanicsOpenEditorOnUpdate','off');file=fullfile(here,'balance_car_lqi.slx');save_system(h,file);model=get_param(h,'Name');
names={'recover','push','bias'};indices=[1 5 7];durations=[6 9 18];summaries=cell(1,3);
for k=1:3
 cfg(1)=indices(k);assignin(mw,'suite_config',cfg);assignin(mw,'terrain_on',0);
 initial=rad2deg(d.pitch_trim);if k==1,initial=initial+5;end
 set_param([model '/FloatingBase'],'SphPositionTargetRotationAngle',num2str(initial,16));set_param(model,'StopTime',num2str(durations(k)));
 o=sim(model);t=o.control3d.Time;c=squeeze(o.control3d.Data);if size(c,1)~=numel(t),c=c';end
 tail=t>=t(end)-1;e=c(:,3:8)-c(:,9:14);
 report=struct('controller','LQI','scenario',names{k},'duration_s',t(end),'fell',any(c(:,21)>.5),...
  'peak_pitch_deg',max(abs(rad2deg(c(:,5)))),'tail_position_error_m',mean(e(tail,1)),...
  'tail_yaw_error_deg',rad2deg(mean(e(tail,5))),'peak_each_current_A',max(abs(c(:,22:23)),[],'all'),...
  'saturation_fraction',mean(any(abs(c(:,22:23))>=2-1e-6,2)),...
  'scope','Native CAD Multibody; ideal current tracking, no measured IMU or FOC dynamics. Separate planar suite screens those uncertainties.');
 summaries{k}=report;out=fullfile(here,'results',['multibody_' names{k}]);save([out '.mat'],'o','report','cfg','d','-v7.3');
 writetable(array2table([t c]),[out '.csv']);disp(report);
end
save_system(model,file);fid=fopen(fullfile(here,'results','multibody_summary.json'),'w');fprintf(fid,'%s',jsonencode(summaries,PrettyPrint=true));fclose(fid);
end
