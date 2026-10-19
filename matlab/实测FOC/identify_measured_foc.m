function f=identify_measured_foc
% Fit effective low-frequency R/Ke, retain LCR inductance as an independent prior.
here=fileparts(mfilename('fullpath'));root=fileparts(fileparts(here));data=fullfile(root,'esp32','current_loop','pi_tuning');
static={'base_d0p1','base_d0p5'};train={'base_w5','base_w10'};holdout={'base_w27p5','base_wm27p5'};
rr=[];vv=[];sig=[];resistancePoints=[];
for n=static
 [T,J,t,bus,vd,vq]=readrun(data,n{1});
 for m=1:2
  I=T.motor==m&t>.14&t<.19&bitand(uint32(T.flags),uint32(63))==63;
  a=mean(T.id(I));b=mean(vd(I));rr(end+1)=a;vv(end+1)=b;
  resistancePoints(end+1,:)=[m a b b/a];
  for ch=1:2
   raw=T.raw_a(I);if ch==2,raw=T.raw_b(I);end
   lut=J.lut_mv.(['x' num2str(2*(m-1)+ch-1)]); % jsondecode makes numeric keys valid names
   phase=(double(lut(raw+1))*.001-J.setup.offset_V(2*(m-1)+ch))*J.setup.sense_gain(m);
   sig(end+1)=median(abs(diff(phase)))/.9538725524;
  end
 end
end
R=rr(:)\vv(:);L=24e-6;A=[];Y=[];points=[];
for n=[train holdout]
 [T,J,t,bus,vd,vq]=readrun(data,n{1});
 for m=1:2
  I=T.motor==m&T.stage==2&t>1;
  w=mean(T.speed(I));iq=mean(T.iq(I));id=mean(T.id(I));u=mean(vq(I));
  corrected=u-R*iq-7*L*mean(T.speed(I).*T.id(I));
  istrain=any(strcmp(n{1},train));if istrain,A(end+1,1)=w;Y(end+1,1)=corrected;end
  points(end+1,:)=[m w iq id u corrected double(istrain)];
 end
end
ke=A\Y;assert(R>.03&&R<.25&&ke>.002&&ke<.02,'Fit outside plausible bounds; inspect reconstruction');
B=zeros(1,2);Tc=B;
for m=1:2
 I=points(:,1)==m;x=points(I,2);y=1.5*ke*points(I,3);
 v=lsqnonneg([x tanh(x/.5)],y);B(m)=v(1);Tc(m)=v(2);
end
prediction=R*points(:,3)+7*L*points(:,2).*points(:,4)+ke*points(:,2);
f=struct('R_effective_ohm',R,'L_prior_H',L,'ke_effective_V_per_rad_s',ke, ...
 'Kt_conditional_Nm_per_A',1.5*ke,'noise_sigma_AB_A',median(sig), ...
 'B_Nm_per_rad_s',B,'coulomb_Nm',Tc,'R_points',resistancePoints, ...
 'speed_points_columns',{{'motor','omega','iq','id','vq_applied','vq_corrected','training'}}, ...
 'speed_points',points,'holdout_vq_rmse_V',sqrt(mean((prediction(points(:,7)==0)-points(points(:,7)==0,5)).^2)), ...
 'sources',struct('R','PWM reconstruction plus static 0.10/0.50A plateaus', ...
 'ke','5/10rad/s electrical operating points; fast positive/negative held out', ...
 'L','user terminal LCR 48uH /2; not newly identified', ...
 'noise','robust phase-current successive difference, includes control ripple', ...
 'friction','conditional plateau torque balance given fitted ke; not external torque calibration'), ...
 'absolute_current_calibrated',false,'switching_waveforms_validated',false);
file=fullfile(here,'identified_parameters.json');fid=fopen(file,'w');assert(fid>=0);fprintf(fid,'%s',jsonencode(f,PrettyPrint=true));fclose(fid);
writetable(array2table([points prediction prediction-points(:,5)],'VariableNames', ...
 {'motor','speed_rad_s','iq_A','id_A','vq_applied_V','vq_corrected_V','training','vq_predicted_V','residual_V'}),fullfile(here,'electrical_fit_points.csv'));
disp(f);fprintf('IDENTIFICATION_COMPLETE R=%g ke=%g noise=%g heldout_vq_rmse=%g\n',R,ke,f.noise_sigma_AB_A,f.holdout_vq_rmse_V);
end
function [T,J,t,bus,vd,vq]=readrun(folder,n)
T=readtable(fullfile(folder,[n '.csv']));J=jsondecode(fileread(fullfile(folder,[n '.json'])));t=(T.us-J.meta.profile_start)*1e-6;
bus=double(J.lut_mv.x4(T.raw_bus+1))*.003;
z=(T.ca+T.cb+T.cc)/3;va=bus.*(T.ca-z)/4000;vb=bus.*(T.cb-z)/4000;vc=bus.*(T.cc-z)/4000;
alpha=va;beta=(vb-vc)/sqrt(3);vd=alpha.*cos(T.theta)+beta.*sin(T.theta);vq=-alpha.*sin(T.theta)+beta.*cos(T.theta);
end
