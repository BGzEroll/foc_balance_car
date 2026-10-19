function p=measured_sensor_parameters
here=fileparts(mfilename('fullpath'));p=struct('enabled',true,'rho_acc',.3,'rho_gyro',.3, ...
 'gyro_residual_bias_rad_s',[0 0 0],'acc_cov_g2',eye(3)*1e-6, ...
 'gyro_cov_rad2_s2',eye(3)*2.5e-7,'Ts',.005,'bandwidth_hz',44,'radius',.0325, ...
 'acc_scale_10',1,'acc_scale_20',1,'gyro_scale_10',1,'gyro_scale_20',1,'fit_loaded',false);
file=fullfile(here,'sensor_noise_parameters.json');
if isfile(file)
 f=jsondecode(fileread(file));p.rho_acc=f.rho_acc;p.rho_gyro=f.rho_gyro;
 p.acc_cov_g2=f.acc_cov_g2;p.gyro_cov_rad2_s2=f.gyro_cov_rad2_s2;p.gyro_residual_bias_rad_s=f.gyro_residual_bias_rad_s(:)';
 p.acc_scale_10=f.acc_scale_10;p.acc_scale_20=f.acc_scale_20;p.gyro_scale_10=f.gyro_scale_10;p.gyro_scale_20=f.gyro_scale_20;p.fit_loaded=true;
end
L1=chol(p.acc_cov_g2+eye(3)*1e-14,'lower');L2=chol(p.gyro_cov_rad2_s2+eye(3)*1e-14,'lower');
p.cfg=[double(p.enabled) p.rho_acc p.rho_gyro p.gyro_residual_bias_rad_s p.Ts p.bandwidth_hz p.radius L1(:)' L2(:)' p.acc_scale_10 p.acc_scale_20 p.gyro_scale_10 p.gyro_scale_20];
La10=L1;La20=L1;Lg10=L2;Lg20=L2;rho=[p.rho_acc p.rho_acc p.rho_gyro p.rho_gyro];gain=1;
if p.fit_loaded
 La10=chol(f.acc_cov_10_g2+eye(3)*1e-14,'lower');La20=chol(f.acc_cov_20_g2+eye(3)*1e-14,'lower');
 Lg10=chol(f.gyro_cov_10_rad2_s2+eye(3)*1e-14,'lower');Lg20=chol(f.gyro_cov_20_rad2_s2+eye(3)*1e-14,'lower');
 rho=[f.rho_acc_10 f.rho_acc_20 f.rho_gyro_10 f.rho_gyro_20];
 if isfield(f,'acc_common_scale_single_pose_prior'),gain=f.acc_common_scale_single_pose_prior;end
end
p.cfg=[p.cfg La10(:)' La20(:)' Lg10(:)' Lg20(:)' rho gain];
for family={'acc','gyro'}
 for profile={'static','10','20'}
  name=[family{1} '_ar_' profile{1}];A=zeros(12,3);
  if p.fit_loaded&&isfield(f,name),ar=f.(name);A=ar.coefficients;L=ar.innovation_chol;
  else
   rho0=p.rho_acc;L=L1;if strcmp(family{1},'gyro'),rho0=p.rho_gyro;L=L2;end
   A(1,:)=-rho0;L=L*sqrt(1-rho0^2);
  end
  p.cfg=[p.cfg A(:)' L(:)'];
 end
end
end
