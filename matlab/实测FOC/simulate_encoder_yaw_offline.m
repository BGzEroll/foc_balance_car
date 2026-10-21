function rows=simulate_encoder_yaw_offline(out)
% Fast open-loop tests: same input sequence for all three observers.
% Deliberate slow-turn/slip tests expose the encoder-only detector boundary.
dt=.005;t=(0:dt:120)';n=numel(t);p=encoder_yaw_parameters;rng(20261021);
rate=zeros(n,1);rate(t>=10&t<14)=.15;rate(t>=25&t<29)=-.15;
rate(t>=60&t<70)=.003;rate(t>=85&t<90)=.05;
truth=cumsum(rate)*dt;encRate=rate;encRate(t>=85&t<90)=0;encYaw=cumsum(encRate)*dt;
common=2*t+.1*sin(t);left=common-p.track_m/(2*p.radius_m)*encYaw;right=common+p.track_m/(2*p.radius_m)*encYaw;
quant=2*pi/4096;encAngle=round([left right]/quant+.354*randn(n,2))*quant;
encSpeed=zeros(n,2);for k=2:n,encSpeed(k,:)=encSpeed(k-1,:)+dt/(.003+dt)*((encAngle(k,:)-encAngle(k-1,:))/dt-encSpeed(k-1,:));end
noise=zeros(n,1);z=randn(n,1);for k=2:n,noise(k)=.42*noise(k-1)+sqrt(1-.42^2)*.0006*z(k);end
rows=struct([]);profiles={'nominal','bias_drift'};
f=figure('Visible','off','Color','w','Position',[30 30 1450 950]);tl=tiledlayout(3,2,'Padding','compact');
for c=1:2
 bias=ones(n,1)*.00015843402400953771;
 if c==2,bias=bias+deg2rad(.25+.003*t);end
 gyro=rate+bias+noise;states=zeros(48,3);states(3,:)=p.bias_initial_std_rad_s^2;
 logs=zeros(n,12,3);
 for k=1:n
  enc=[encSpeed(k,:) encAngle(k,:)]';
  for m=1:3,[states(:,m),logs(k,:,m)]=step(states(:,m),gyro(k),enc,dt,m);end
 end
 labels={'A 始终积分+零偏LP','B 航向保持','C 保持+零速KF'};
 for m=1:3
  estimate=logs(:,1,m);gate=logs(:,7,m)>.5;holdMask=gate&(m>=2);
  normal=t<55;movingNormal=normal&abs(rate)>.02;slow=t>=60&t<70;slip=t>=85&t<90;
  row=struct('profile',profiles{c},'mode',char('A'+m-1), ...
   'normal_rms_yaw_error_deg',rad2deg(sqrt(mean((estimate(normal)-truth(normal)).^2))), ...
   'error_at_55s_deg',rad2deg(estimate(find(t>=55,1))-truth(find(t>=55,1))), ...
   'normal_false_hold_s',sum(holdMask&movingNormal)*dt, ...
   'slow_turn_false_hold_s',sum(holdMask&slow)*dt,'slip_false_hold_s',sum(holdMask&slip)*dt, ...
   'normal_bias_rms_deg_s',rad2deg(sqrt(mean((logs(normal,4,m)-bias(normal)).^2))));
  if isempty(rows),rows=row;else,rows(end+1)=row;end
 end
 nexttile(c);hold on;for m=1:3,plot(t,rad2deg(logs(:,1,m)-truth),'DisplayName',labels{m});end
 xline(60,':','慢转');xline(85,':','滑动反例');grid on;legend('Location','best');ylabel('航向误差 / °');title(profiles{c});
 nexttile(c+2);hold on;for m=1:3,plot(t,rad2deg(logs(:,4,m)),'DisplayName',labels{m});end
 plot(t,rad2deg(bias),'k--','DisplayName','注入真值');grid on;ylabel('估计零偏 / °/s');legend('Location','best');
 nexttile(c+4);plot(t,rad2deg(rate),t,logs(:,7,3)*10);grid on;ylabel('真角速度 / °/s，判定×10');xlabel('时间 / s');legend('真实偏航角速度','编码器确认未转向');
 save(fullfile(out,['offline_' profiles{c} '.mat']),'t','rate','truth','bias','gyro','encAngle','encSpeed','logs','-v7.3');
end
title(tl,'同输入开环A/B/C：常规运动 + 低于阈值慢转 + 编码器无法感知的转动（非机械闭环）');
exportgraphics(f,fullfile(out,'offline.png'),'Resolution',150);savefig(f,fullfile(out,'offline.fig'));close(f);
writetable(struct2table(rows),fullfile(out,'offline_metrics.csv'));disp(struct2table(rows));
end
function [state,d]=step(state,gyro,enc,dt,mode)
[state,d]=measured_encoder_yaw_step(state,gyro,enc,dt,mode);d=d';
end
