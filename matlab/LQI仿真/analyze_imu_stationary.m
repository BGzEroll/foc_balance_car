function report=analyze_imu_stationary(folder)
% Time audit and blocked holdout screening; no fitted calibration is deployed.
here=fileparts(mfilename('fullpath'));a=readtable(fullfile(folder,'imu.csv'),'TextType','string');a=a(a.stage=="static",:);
b=readtable(fullfile(folder,'telemetry.csv'),'TextType','string');b=b(b.stage=="static",:);
capture=jsondecode(fileread(fullfile(folder,'capture.json')));
t=(double(a.time_us)-double(a.time_us(1)))*1e-6;
gyro=[a.gyro_x a.gyro_y a.gyro_z]*180/pi;temp=double(a.temperature_raw)/340+36.53;
tx=double(a.time_us)-double(a.read_began_us);native=double(a.native_sequence);
timing=struct('recorded_samples',height(a),'duration_s',t(end),...
 'native_acquisitions_span',native(end)-native(1)+1,...
 'native_mean_period_us',(double(a.time_us(end))-double(a.time_us(1)))/(native(end)-native(1)),...
 'native_samples_not_in_stream',sum(max(0,diff(native)-1)),...
 'recorded_interval_us_percentiles',prctile(diff(double(a.time_us)),[0 50 95 99 100]),...
 'read_transaction_us_percentiles',prctile(tx,[0 50 95 99 100]),...
 'control_imu_age_us_percentiles',prctile(b.imu_age_us,[0 50 95 99 100]),...
 'control_dt_us_percentiles',prctile(b.dt_us,[0 50 95 99 100]),...
 'control_frames',height(b),'control_reused_imu_count',sum(diff(b.imu_us)==0),...
 'control_reused_imu_fraction',mean(diff(b.imu_us)==0),'crc_errors',capture.crc_errors,...
 'wire_sequence_gaps',capture.missing_stream_sequences);
assert(all(tx>0) && all(diff(native)>0),'Non-monotonic native stream or invalid read timestamp');
bins=floor(t/10);u=unique(bins);u=u(arrayfun(@(v)range(t(bins==v))>=8,u));block=struct([]);
for k=1:numel(u)
 mask=bins==u(k);block(k).time_s=mean(t(mask));block(k).temperature_C=mean(temp(mask));
 block(k).gyro_mean_deg_s=mean(gyro(mask,:),1);block(k).gyro_std_deg_s=std(gyro(mask,:),0,1);
end
validation=t>=max(t(end)-120,t(end)*.75);baseMean=mean(gyro(validation,:),1);
cal=cell(1,15);i=0;
for wait=[0 30 60 120 300]
 for window=[5 10 30]
  if wait+window>=min(t(validation)),continue;end
  train=t>=wait & t<wait+window;offset=mean(gyro(train,:),1);residual=gyro(validation,:)-offset;i=i+1;
  cal{i}=struct('wait_after_first_log_s',wait,'average_window_s',window,'sample_count',sum(train),...
   'heldout_bias_deg_s',mean(residual,1),'heldout_rms_deg_s',rms(residual,1),'estimated_extra_bias_deg_s',offset);
 end
end
cal=[cal{1:i}];
train=t<t(end)*.5;reference=mean(gyro(train,:),1);coef=zeros(2,3);after=zeros(1,3);before=after;
for axis=1:3
 coef(:,axis)=polyfit(temp(train),gyro(train,axis),1)';
 before(axis)=rms(gyro(validation,axis)-reference(axis));
 after(axis)=rms(gyro(validation,axis)-polyval(coef(:,axis),temp(validation)));
end
thermal=struct('temperature_range_C',[min(temp),max(temp)],'fit_slope_deg_s_C',coef(1,:),...
 'heldout_rms_constant_bias_deg_s',before,'heldout_rms_temperature_fit_deg_s',after,'applied',false,...
 'scope','First-half fit, final-quarter holdout. Single fixed-stand warm-up confounds time with temperature; no temperature correction is deployed.');
nativeGrid=(native(1):native(end))';uniform=interp1(native,gyro,nativeGrid);dt=(t(end)-t(1))/(numel(nativeGrid)-1);
taus=[.005 .01 .02 .05 .1 .2 .5 1 2 5 10 20];adev=nan(numel(taus),3);
for k=1:numel(taus)
 m=max(1,round(taus(k)/dt));nb=floor(size(uniform,1)/m);
 if nb<4,continue;end
 for axis=1:3,means=mean(reshape(uniform(1:m*nb,axis),m,nb),1);adev(k,axis)=sqrt(mean(diff(means).^2)/2);end
end
allan=struct('average_time_s',taus,'deviation_deg_s',adev,'scope','Non-overlapping Allan deviation. Missing telemetry samples are interpolated by native index. Finite warm fixed-stand record; do not identify long-term bias instability from a single minimum.');
report=struct('source',folder,'capture',capture,'timing',timing,'blocks_10s',block,...
 'actual_startup_calibration_heldout_mean_deg_s',baseMean,'prospective_calibrations',cal,'thermal',thermal,'allan',allan,...
 'scope',['Stationary fixed stand assumed; no independent angle reference. '...
 'Calibration waiting times start at the first logged sample, about six seconds after MCU boot, not at power-on. '...
 'The original 1000-sample startup calibration is assessed from its residual calibrated gyro, not replayed from unrecorded startup raw samples. '...
 'Missing native indices describe telemetry observation loss, not proved missing sensor acquisitions. '...
 'Read-completion timestamps do not measure the chip DLPF delay or internal register-update phase.']);
[~,label]=fileparts(folder);out=fullfile(here,'results/imu_stationary',label);if ~isfolder(out),mkdir(out);end
fid=fopen(fullfile(out,'report.json'),'w');fprintf(fid,'%s',jsonencode(report,PrettyPrint=true));fclose(fid);
writetable(struct2table(cal),fullfile(out,'calibration_windows.csv'));
fig=figure('Visible','off','Position',[100 100 1450 900]);tiledlayout(2,2,'TileSpacing','compact');
bt=[block.time_s];bg=vertcat(block.gyro_mean_deg_s);
nexttile;plot(t,temp);grid on;xlabel('记录开始后 (s)');ylabel('芯片温度 (°C)');title('电机关闭：温度变化');
nexttile;plot(bt,bg,'-o');grid on;xlabel('记录开始后 (s)');ylabel('10秒平均残余角速度 (°/s)');legend('X','Y','Z');title('现有启动零偏校准后的漂移');
nexttile;histogram(b.imu_age_us,40);grid on;xlabel('控制读取时IMU数据年龄 (μs)');ylabel('次数');title(sprintf('控制重复用旧样本 %.3f%%',100*timing.control_reused_imu_fraction));
nexttile;values=vertcat(cal.heldout_bias_deg_s);bar(vecnorm(values,2,2));grid on;xticks(1:numel(cal));
xticklabels(string([cal.wait_after_first_log_s])+"s / "+string([cal.average_window_s])+"s");xtickangle(60);ylabel('验证段三轴偏置模长 (°/s)');title('等待时间 / 平均窗口：同一末段验证');
sgtitle('IMU静止长采集：时序与零偏筛选，未应用拟合参数');
exportgraphics(fig,fullfile(out,'stationary_audit_full_blocks.png'),'Resolution',160);close(fig);
disp(timing);disp(thermal);
end
