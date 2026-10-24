function compare_1024_cogging(dataDir)
if nargin==0, dataDir=fullfile(fileparts(fileparts(mfilename('fullpath'))),'results','anticogging_1024'); end
previous=fullfile(fileparts(dataDir),'anticogging');
a=load(fullfile(previous,'cogging_calibration.mat'),'map','metrics');
b=load(fullfile(dataDir,'cogging_calibration.mat'),'map','metrics');
c=load(fullfile(previous,'dynamic_cogging.mat'),'map');
angle=(0:4095)'*360/4096;
f=figure('Visible','off','Color','w','Position',[50 50 1450 850]); tiledlayout(2,2); metrics=struct([]);
for m=1:2
    nexttile; plot(angle,a.map(:,m)); hold on; plot(angle,b.map(:,m)); grid on;
    xlabel('机械角度 (°)'); ylabel('补偿电流 Iq (A)'); title(sprintf('M%d：静态实测位置加密',m-1));
    legend('360位置拟合','1024位置拟合','Location','best');
    nexttile; plot(angle,b.map(:,m)-a.map(:,m)); hold on; plot(angle,b.map(:,m)-c.map(:,m)); grid on;
    xlabel('机械角度 (°)'); ylabel('补偿表差值 (A)'); title(sprintf('M%d：新旧4096项表差值',m-1));
    legend('相对360位置静态表','相对先前运动修正版','Location','best');
    metrics(m).motor=m-1;
    metrics(m).old_static_fit_rms_A=a.metrics(m).fit_rms_A;
    metrics(m).new_static_fit_rms_A=b.metrics(m).fit_rms_A;
    metrics(m).old_direction_difference_rms_A=a.metrics(m).direction_difference_rms_A;
    metrics(m).new_direction_difference_rms_A=b.metrics(m).direction_difference_rms_A;
    metrics(m).change_from_old_static_rms_A=rms(b.map(:,m)-a.map(:,m));
    metrics(m).change_from_previous_deployed_rms_A=rms(b.map(:,m)-c.map(:,m));
end
fid=fopen(fullfile(dataDir,'comparison_1024_metrics.json'),'w'); fprintf(fid,'%s',jsonencode(metrics,PrettyPrint=true)); fclose(fid);
exportgraphics(f,fullfile(dataDir,'comparison_1024_calibration.png'),'Resolution',160);
disp(metrics);
end
