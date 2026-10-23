addpath('D:/code/Embedded-related/balance_car/matlab/wifi_tuning');
folder='D:/code/Embedded-related/balance_car/esp32/tuning_preparation/sessions/20261023_live_05';
[T,report]=analyze_session(folder,false);
replay_current_pid(folder);
active=T.mode==2 & T.fault==0 & bitand(uint32(T.flags),uint32(7))==7;
start=min(T.time_s(active))+2; finish=max(T.time_s(active))-1;
valid=active & T.time_s>=start & T.time_s<=finish & ...
 T.imu_age_us<=20000 & T.encoder_age_us<=1500 & ...
 T.current_age_a_us<1000 & T.current_age_b_us<1000;
V=T(valid,:);
edges=[1;find(diff(V.sample_us)>=15000)+1;height(V)+1];
[n,k]=max(diff(edges));
if n>=600
 window=[V.time_s(edges(k)),V.time_s(edges(k+1)-1)];
 try
  fit_current_plant(folder,window);
  explore_current_pid(fullfile(folder,'identified_plant.mat'));
 catch error
  fprintf('MODEL_REJECTED: %s\n',error.message);
 end
else
 fprintf('MODEL_REJECTED: only %d contiguous valid samples after startup exclusion\n',n);
end
