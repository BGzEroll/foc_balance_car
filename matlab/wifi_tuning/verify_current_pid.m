function verify_current_pid
here=fileparts(mfilename('fullpath'));root=fileparts(fileparts(here));
folder=fullfile(root,'esp32','tuning_preparation');
p=jsondecode(fileread(fullfile(folder,'profiles','wired_B_current.json')));
x=readmatrix(fullfile(folder,'results','equivalence_vectors.csv'));
prev=-1;s=[];maxError=0;
for k=1:size(x,1)
 if x(k,1)~=prev,s=[];prev=x(k,1);end
 g=p.gains;g.velocity_kp=x(k,8);g.velocity_ki=x(k,9);g.pitch_kd=x(k,10);
 m=struct('speed',x(k,2),'pitch',x(k,3),'pitch_rate',x(k,4),'yaw',x(k,5),'yaw_rate',x(k,6));
 [out,s]=current_pid_step(m,s,g,x(k,7));
 maxError=max(maxError,max(abs([out.current,out.target_pitch]-x(k,11:13))));
end
assert(maxError<2e-4,'MATLAB/C++ mismatch');
report=struct('samples',size(x,1),'max_error_A_or_rad',maxError,'controller_period_s',.005);
fid=fopen(fullfile(folder,'results','matlab_equivalence.json'),'w');fprintf(fid,'%s',jsonencode(report));fclose(fid);
disp(report)
end
