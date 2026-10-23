function candidates=explore_current_pid(model_file)
% Local-model screening only. Never sends parameters to the car.
data=load(model_file,'model');model=data.model;
assert(model.valid_for_search,'Model validation gate failed');
assert(isfield(model,'disturbance_validation_passed')&&model.disturbance_validation_passed,...
 'Multi-step validation on a separate time segment is required before gain search');
base=model.baseline;results=[];profiles={};
for vp=[.6 .8 1 1.2]
 for vi=[.5 1 1.5]
  for kd=[.8 1 1.2 1.5]
   g=base;g.velocity_kp=base.velocity_kp*vp;g.velocity_ki=base.velocity_ki*vi;g.pitch_kd=base.pitch_kd*kd;
   worst=0;fallen=false;saturation=0;
   for initial=[-2,2]
    x=[deg2rad(initial);0;0];s=[];previous_current=0;energy=0;nsat=0;
    for k=1:1200
     m=struct('pitch',x(1)+g.pitch_trim_rad,'pitch_rate',x(2),'speed',x(3),'yaw',0,'yaw_rate',0);
     [out,s]=current_pid_step(m,s,g,model.current_limit_A);
     % Measured current at cycle k precedes command k: include one-cycle delay.
     f=@(z)model.A*z+model.B*previous_current+model.bias;
     h=.005;a=f(x);b=f(x+h*a/2);c=f(x+h*b/2);d=f(x+h*c);x=x+h*(a+2*b+2*c+d)/6;
     previous_current=sum(out.current);nsat=nsat+out.saturated;
     energy=energy+x(1)^2+.2*x(3)^2+.002*sum(out.current.^2);
     if any(~isfinite(x))||abs(x(1))>deg2rad(15)||abs(x(3))>.4,fallen=true;break;end
    end
    worst=max(worst,energy);saturation=max(saturation,nsat/k);
   end
   if fallen,worst=Inf;end
   results(end+1,:)=[vp vi kd worst saturation];profiles{end+1}=g; %#ok<AGROW>
  end
 end
end
[~,order]=sort(results(:,4));results=results(order,:);profiles=profiles(order);
candidates=array2table(results,'VariableNames',{'velocity_P_scale','velocity_I_scale','pitch_D_scale','cost','saturation_fraction'});
folder=fileparts(model_file);writetable(candidates,fullfile(folder,'simulation_candidates.csv'));
candidate_path=fullfile(folder,'candidate_review_only.json');
if isfinite(results(1,4))
 p=struct('schema','current_pid_v1','gains',profiles{1},'current_limit_A',model.current_limit_A,...
  'source','OFFLINE CANDIDATE ONLY; needs human review and powered test','model_holdout_nrmse',model.holdout_nrmse);
else
 p=struct('schema','no_feasible_candidate','source','All local-model candidates rejected; do not apply prior results');
end
fid=fopen(candidate_path,'w');fprintf(fid,'%s',jsonencode(p));fclose(fid);
disp(candidates(1:min(8,height(candidates)),:))
end
