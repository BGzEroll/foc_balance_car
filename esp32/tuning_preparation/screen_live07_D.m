addpath('D:/code/Embedded-related/balance_car/matlab/wifi_tuning');
folder='D:/code/Embedded-related/balance_car/esp32/tuning_preparation/sessions/20261023_live_07';
[T,~]=analyze_session(folder,false);
names={'velocity_kp','velocity_ki','pitch_kd','pitch_kp','pitch_ki','yaw_kp','yaw_ki','yaw_kd','position_kp','pitch_trim_rad'};
commands=[]; saturated=[];
for k=2:height(T)
 if T.mode(k)~=2 || T.mode(k-1)~=2 || T.fault(k)~=0 || T.cycle(k)-T.cycle(k-1)~=1, continue; end
 s=struct('distance',T.distance(k-1),'prev_speed',T.speed(k-1),...
  'vi',T.velocity_integral(k-1),'pi',T.pitch_integral(k-1),'yi',T.yaw_integral(k-1),'first',false);
 g=struct; for j=1:numel(names),g.(names{j})=T.(names{j})(k);end
 m=struct('pitch',T.pitch(k),'pitch_rate',T.pitch_rate(k),'speed',T.speed(k),...
  'yaw',T.yaw(k),'yaw_rate',T.yaw_rate(k));
 [a,~]=current_pid_step(m,s,g,2);g.pitch_kd=g.pitch_kd/2;
 [b,~]=current_pid_step(m,s,g,2);
 commands(end+1,:)=[a.current,b.current];saturated(end+1,:)=[a.saturated,b.saturated]; %#ok<SAGROW>
end
report=struct('pairs',height(commands),'target_limit_A',2,...
 'baseline_rms_A',sqrt(mean(commands(:,1:2).^2)),...
 'half_D_rms_A',sqrt(mean(commands(:,3:4).^2)),...
 'saturation_fraction',[mean(saturated(:,1)),mean(saturated(:,2))],...
 'interpretation','Frozen recorded observations and preceding actual integrators. Command sensitivity only; not a closed-loop simulation or stability prediction.');
fid=fopen(fullfile(folder,'D_half_screen.json'),'w');fprintf(fid,'%s',jsonencode(report));fclose(fid);disp(report);
