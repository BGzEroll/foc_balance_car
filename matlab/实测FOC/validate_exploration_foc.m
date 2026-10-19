function r=validate_exploration_foc
% Model stress test only, not new hardware evidence or an identified L value.
[~,~,p]=measured_pid_config('baseline',10,'explore6');r=struct([]);
for bus=[6.4 8.03]
 for scale=[1 .5 1.5]
  for amp=[.7 3 6 10]
   cfg=p.cfg;cfg(9)=bus;cfg(2)=p.Ld*scale;cfg(12)=max(6,amp);x=measured_foc_initial([0 0]);w=[40 40];last=zeros(1000,2);peak=0;startupPeak=0;
   for k=0:3999
    ref=zeros(2,2);if k>=1000,ref(2,:)=amp;end
    [x,y]=measured_foc_step(x,ref,w*k*p.Ts,w,k,cfg);
    if k>=1000,peak=max(peak,max(abs(y(3,:))));else,startupPeak=max(startupPeak,max(abs(y(3,:))));end
    if k>=3000,last(k-2999,:)=y(3,:);end
    if peak>100,break;end
   end
   row=struct('bus_V',bus,'L_prior_multiplier',scale,'Iq_target_A',amp,'speed_rad_s',40,'warmup_s',.05,'uninitialized_spinning_state_peak_A',startupPeak,'peak_after_current_command_A',peak,'steady_actual_mean_A',mean(last),'steady_actual_std_A',std(last), ...
    'pass',peak<amp*1.4+.3&&all(abs(mean(last)-amp)<.15)&&all(std(last)<.2));
   if isempty(r),r=row;else,r(end+1)=row;end
  end
 end
end
here=fileparts(mfilename('fullpath'));out=fullfile(here,'pid_validation');if ~isfolder(out),mkdir(out);end
fid=fopen(fullfile(out,'exploration_foc.json'),'w');fprintf(fid,'%s',jsonencode(r,PrettyPrint=true));fclose(fid);
disp(r);assert(all([r([r.L_prior_multiplier]==1).pass]),'Nominal electrical model failed exploration stress test');
end
