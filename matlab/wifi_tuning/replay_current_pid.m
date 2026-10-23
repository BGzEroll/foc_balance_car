function report=replay_current_pid(folder)
% One-step controller replay using the preceding actual integrator state.
% Gaps and non-balance modes are excluded, never interpolated as controller truth.
[T,~]=analyze_session(folder);
names={'velocity_kp','velocity_ki','pitch_kd','pitch_kp','pitch_ki','yaw_kp','yaw_ki','yaw_kd','position_kp','pitch_trim_rad'};
errors=[];
for k=2:height(T)
 if T.mode(k)~=2||T.mode(k-1)~=2||T.cycle(k)-T.cycle(k-1)~=1,continue;end
 s=struct('distance',T.distance(k-1),'prev_speed',T.speed(k-1),...
  'vi',T.velocity_integral(k-1),'pi',T.pitch_integral(k-1),'yi',T.yaw_integral(k-1),'first',false);
 g=struct;for j=1:numel(names),g.(names{j})=T.(names{j})(k);end
 m=struct('pitch',T.pitch(k),'pitch_rate',T.pitch_rate(k),'speed',T.speed(k),...
  'yaw',T.yaw(k),'yaw_rate',T.yaw_rate(k));
 [out,~]=current_pid_step(m,s,g,T.current_limit_A(k));
 errors(end+1,:)=abs([out.current,out.target_pitch]-[T.command_a(k),T.command_b(k),T.target_pitch(k)]); %#ok<AGROW>
end
report=struct('pairs',size(errors,1));
if ~isempty(errors),report.max_error=max(errors,[],1);report.rms_error=sqrt(mean(errors.^2,1));
else,report.note='No consecutive active balance cycles; idle capture cannot verify runtime control';end
fid=fopen(fullfile(folder,'controller_replay.json'),'w');fprintf(fid,'%s',jsonencode(report));fclose(fid);disp(report)
end
