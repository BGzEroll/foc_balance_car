% LQI专用入口。默认展示已验证结果；不存在结果时先离线运行。
here=fileparts(mfilename('fullpath'));addpath(here);
if ~isfile(fullfile(here,'results','lqi_suite.mat')),run_lqi_suite(false);end
plot_lqi_results;
if isfile(fullfile(here,'results','multibody_bias.mat')),plot_lqi_multibody;end
f=fullfile(here,'balance_car_lqi.slx');
if isfile(f)
 root=fileparts(fileparts(here));addpath(fullfile(root,'matlab'),fullfile(root,'matlab','3D仿真'));balance_setup;
 load_system(f);model='balance_car_lqi';[cfg,d]=balance_control_config('lqi',1,2,[1 1],'normal');
 mw=get_param(model,'ModelWorkspace');assignin(mw,'suite_config',cfg);assignin(mw,'terrain_on',0);
 set_param([model '/FloatingBase'],'SphPositionTargetRotationAngle',num2str(rad2deg(d.pitch_trim)+5,16));
 set_param(model,'StopTime','6','SimMechanicsOpenEditorOnUpdate','on');open_system(model);
end
