function p=measured_foc_parameters
% Averaged actuator calibrated to v18 data; not a switching digital twin.
here=fileparts(mfilename('fullpath'));root=fileparts(fileparts(here));
p=struct('pole_pairs',7,'Rs',.09,'Ld',24e-6,'Lq',24e-6, ...
 'ke',60/(2*pi*sqrt(3)*700),'Kp',.060318579,'Ki',226.194672, ...
 'Ts',50e-6,'control_period',200e-6,'output_delay',150e-6, ...
 'mechanical_macro_step',.00025, ...
 'Vbus',8.03,'ratio',.04,'voltage_cap',.4,'Imax',.7, ...
 'noise_sigma_AB',.012,'encoder_counts',4096,'encoder_latency',200e-6, ...
 'J_bare',5.165958224724388e-6,'J_wheel',2.996788998409844e-5, ...
 'B',[1e-5 1e-5],'coulomb',[0 0],'fit_loaded',false);
fit=fullfile(here,'identified_parameters.json');
if isfile(fit)
 f=jsondecode(fileread(fit));p.Rs=f.R_effective_ohm;p.ke=f.ke_effective_V_per_rad_s;
 p.noise_sigma_AB=f.noise_sigma_AB_A;p.B=f.B_Nm_per_rad_s(:)';p.coulomb=f.coulomb_Nm(:)';p.fit_loaded=true;
end
p.flux=p.ke/p.pole_pairs;p.Kt=1.5*p.ke;
p.root=root;p.data=fullfile(root,'esp32','current_loop','pi_tuning');
covAB=repmat(eye(2)*p.noise_sigma_AB^2,1,1,2);
if p.fit_loaded&&isfield(f,'effective_cov_AB_A2'),covAB=f.effective_cov_AB_A2;end
chols=zeros(2,2,2);for m=1:2,chols(:,:,m)=chol(covAB(:,:,m)+eye(2)*1e-14,'lower');end
encoderJitter=[.354 .354];sensorFile=fullfile(here,'sensor_noise_parameters.json');
if isfile(sensorFile),ns=jsondecode(fileread(sensorFile));encoderJitter(:)=max(ns.encoder_jitter_counts);end
assert(all([p.Rs p.Ld p.Lq p.Kp p.Ki p.Ts p.Vbus p.ke]>0));
% cfg: R L ke Kp Ki dt controlTicks delayTicks bus ratio cap Imax noise
%      encoderCounts encoderDelayTicks polePairs B_L B_R Tc_L Tc_R
p.cfg=[p.Rs p.Ld p.ke p.Kp p.Ki p.Ts round(p.control_period/p.Ts) ...
 round(p.output_delay/p.Ts) p.Vbus p.ratio p.voltage_cap p.Imax ...
 p.noise_sigma_AB p.encoder_counts round(p.encoder_latency/p.Ts) p.pole_pairs p.B p.coulomb chols(:)' encoderJitter round(p.mechanical_macro_step/p.Ts)];
end
