function verify_reference
% Generate expected results by executing the unchanged MATLAB controller.
here=fileparts(mfilename('fullpath'));addpath(fullfile(here,'reference'));
folder=fullfile(here,'..','balance_car','build','upper_controller');
if ~isfolder(folder),mkdir(folder);end
inputs=[];expected=[];
for group=1:6
 clear measured_car_pid
 scenario=1;if group==3||group==6,scenario=3;end
 if group==2||group==5,scenario=4;end
 [cfg,~,p]=measured_pid_config('selected',scenario,'bench',.005);
 if group>3,cfg(2)=.25;end
 for k=1:2400
  t=(k-1)*.005;
  % Small unsaturated inputs, followed by saturation and wrapped yaw.
  scale=1;if t<2,scale=.002;elseif t<4,scale=.02;end
  pitch=scale*.12*sin(1.7*t);pd=scale*.204*cos(1.7*t);
  yaw=scale*3.4*sin(.7*t);yd=scale*2.38*cos(.7*t);
  v=scale*.4*sin(.9*t);
  qw=cos(yaw/2)*cos(pitch/2);qx=-sin(yaw/2)*sin(pitch/2);
  qy=cos(yaw/2)*sin(pitch/2);qz=sin(yaw/2)*cos(pitch/2);
  R=[cos(yaw)*cos(pitch),-sin(yaw),cos(yaw)*sin(pitch);...
     sin(yaw)*cos(pitch),cos(yaw),sin(yaw)*sin(pitch);...
     -sin(pitch),0,cos(pitch)];
  wb=[-yd*sin(pitch);pd;yd*cos(pitch)];ww=R*wb;
  raw=[t;0;-sin(yaw)*v;0;cos(yaw)*v;0;0;qw;-qy;qx;qz;-ww(2);ww(1);ww(3)];
  out=measured_car_pid(raw,cfg,zeros(0,4));
  [ref,~,~]=measured_car_reference(t,scenario,zeros(0,4));
  % Match the actual observations reconstructed by MATLAB, not ideal angles.
  measured=[out(4),out(5)+cfg(3),out(6),out(7),out(8)];
  inputs(end+1,:)=[group,measured,ref(1),ref(2),ref(5),ref(6),cfg(2)]; %#ok<AGROW>
  expected(end+1,:)=[out(22:23)',out(24),out(25),out(3)]; %#ok<AGROW>
 end
end
writematrix(inputs,fullfile(folder,'inputs.csv'));
writematrix(expected,fullfile(folder,'expected.csv'));
adapterInputs=zeros(6000,8);adapterExpected=zeros(6000,5);
clear measured_car_pid
for k=1:6000
 t=(k-1)*.005;
 a=.3*sin(1.3*t);b=.25*sin(.7*t);c=3.8*sin(.4*t);
 g=[.6*cos(.8*t);.5*sin(.5*t);.7*cos(.6*t)];
 wheels=[8*sin(.9*t);10*sin(.6*t)];
 ca=cos(a/2);sa=sin(a/2);cb=cos(b/2);sb=sin(b/2);cc=cos(c/2);sc=sin(c/2);
 q=[ca*cb*cc+sa*sb*sc;sa*cb*cc-ca*sb*sc;ca*sb*cc+sa*cb*sc;ca*cb*sc-sa*sb*cc];
 R=[cos(c)*cos(b),cos(c)*sin(b)*sin(a)-sin(c)*cos(a),cos(c)*sin(b)*cos(a)+sin(c)*sin(a);...
    sin(c)*cos(b),sin(c)*sin(b)*sin(a)+cos(c)*cos(a),sin(c)*sin(b)*cos(a)-cos(c)*sin(a);...
    -sin(b),cos(b)*sin(a),cos(b)*cos(a)];
 forward=.0325*(mean(wheels)-g(1));
 raw=[0;0;-sin(c)*forward;0;cos(c)*forward;0;0;q;R*g];
 out=measured_car_pid(raw,cfg,zeros(0,4));
 adapterInputs(k,:)=[a,b,c,g',wheels'];
 adapterExpected(k,:)=[out(4),out(5)+cfg(3),out(6),out(7),out(8)];
end
writematrix(adapterInputs,fullfile(folder,'adapter_inputs.csv'));
writematrix(adapterExpected,fullfile(folder,'adapter_expected.csv'));
meta=struct('source_branch','test/foc_current','source_commit','207729fa6435fd8a87485c622fdbe5fa7c250d8d',...
 'samples',size(inputs,1),'adapter_samples',6000,'groups',6,'period_s',.005,'Kt_Nm_A',p.Kt);
fid=fopen(fullfile(folder,'reference.json'),'w');fprintf(fid,'%s',jsonencode(meta));fclose(fid);
disp('REFERENCE_GENERATED');
end
