function r=report_foc_demo(o,p,scenario,resultsRoot,showWindows)
out=fullfile(resultsRoot,'foc',scenario);if ~isfolder(out),mkdir(out);end
v=o.foc_data;t=v.Time;y=squeeze(v.Data);if size(y,1)~=numel(t),y=y';end
% x(7), ref(4), u(9), measured(5)
id=y(:,1);iq=y(:,2);omega=y(:,3);ref=y(:,9);load=y(:,10);
duty=y(:,16:18);sat=y(:,19);limited=y(:,20);phase=y(:,21:23);
torque=1.5*p.pole_pairs*(p.flux*iq+(p.Ld-p.Lq)*id.*iq);
stored=.75*(p.Ld*id.^2+p.Lq*iq.^2)+.5*p.J*omega.^2;
balance=y(:,5)-y(:,6)-y(:,7)-(stored-stored(1));
energyResidual=max(abs(balance))/max([max(abs(y(:,5))) max(stored) 1e-9]);
% 只对未饱和且阶跃后至少10ms的样本检查稳态电流跟踪。
settled=true(size(t));changes=[0;find(abs(diff(ref))>1e-9)];
for k=1:numel(changes)
 at=t(max(1,changes(k)+1));settled(t>=at&t<at+.01)=false;
end
settled=settled&sat<.5;
error=iq-limited;
if any(settled),steadyError=max(abs(error(settled)));else,steadyError=Inf;end
firstStep=t>=.02&t<.10;firstBad=find(firstStep&abs(error)>.02*max(abs(limited),1e-9),1,'last');
stepSettling=0;
if ~isempty(firstBad),stepSettling=1000*(t(min(firstBad+1,numel(t)))-.02);end
r=struct('scenario',scenario,'pass',false,'duration_s',t(end), ...
 'first_step_2percent_settling_ms',stepSettling, ...
 'max_settled_iq_error_A',steadyError,'peak_current_magnitude_A',max(hypot(id,iq)), ...
 'peak_id_A',max(abs(id)),'min_speed_rpm',min(omega)*30/pi,'max_speed_rpm',max(omega)*30/pi, ...
 'peak_torque_Nm',max(abs(torque)),'voltage_saturated_fraction',mean(sat>.5), ...
 'energy_relative_residual',energyResidual,'Kt_estimated_Nm_per_A',p.Kt, ...
 'Vdc_V',p.Vdc,'iq_limit_A',p.Imax,'model_is_averaged',true);
basic=all(isfinite(y),'all')&&t(end)>.4&&min(duty,[],'all')>=-1e-12 ...
 &&max(duty,[],'all')<=1+1e-12&&max(abs(sum(phase,2)))<1e-9&&energyResidual<1e-5;
if strcmp(scenario,'voltage')
 r.pass=basic&&r.voltage_saturated_fraction>.05&&max(omega)>200;
else
 r.pass=basic&&steadyError<.05&&r.peak_current_magnitude_A<p.Imax*1.15;
 if strcmp(scenario,'current'),r.pass=r.pass&&max(abs(omega))<1e-12&&max(limited)>=p.Imax;end
 if strcmp(scenario,'reverse'),r.pass=r.pass&&min(omega)<-5&&max(omega)>5&&min(torque)<-.02;end
 if strcmp(scenario,'load')
  before=omega(find(t>=.14,1))-omega(find(t>=.10,1));
  during=omega(find(t>=.24,1))-omega(find(t>=.20,1));
  r.pass=r.pass&&max(load)>.019&&during<before;
 end
end
tab=array2table([t ref limited id iq omega*30/pi torque load y(:,14:15) duty sat phase ...
 y(:,5:7) stored balance], ...
 'VariableNames',{'time_s','iq_requested_A','iq_limited_A','id_A','iq_A','speed_rpm', ...
 'torque_Nm','load_Nm','vd_V','vq_V','duty_A','duty_B','duty_C','voltage_saturated', ...
 'ia_A','ib_A','ic_A','electrical_energy_J','copper_loss_J','load_and_friction_energy_J', ...
 'stored_energy_J','energy_residual_J'});
writetable(tab,fullfile(out,'waveforms.csv'));
save(fullfile(out,'simulation.mat'),'o','p','r','-v7.3');
fid=fopen(fullfile(out,'validation.json'),'w');assert(fid>=0);fprintf(fid,'%s',jsonencode(r,PrettyPrint=true));fclose(fid);
fid=fopen(fullfile(out,'parameters.json'),'w');assert(fid>=0);fprintf(fid,'%s',jsonencode(p,PrettyPrint=true));fclose(fid);
tag='zd2808_foc_results';old=findall(groot,'Type','figure','Tag',tag);delete(old);
visibility='off';if showWindows,visibility='on';end
f=figure('Name',['ZD2808 FOC - ' scenario],'NumberTitle','off','Tag',tag,'Visible',visibility, ...
 'Position',[80 60 1180 830],'Color','w');
tiledlayout(3,2,'Padding','compact');
nexttile;plot(t,ref,':',t,limited,'--',t,iq,t,id);grid on;ylabel('电流 / A');legend('原始iq指令','限流后指令','实际iq','实际id','Location','best');
title('电流闭环：id目标为0');
nexttile;plot(t,torque,t,load,'--');grid on;ylabel('N·m');legend('电磁转矩','外负载');title('实际转矩');
nexttile;
if strcmp(scenario,'current')
 plot((t-.02)*1000,limited,'--',(t-.02)*1000,iq);xlim([-1 4]);grid on;
 ylabel('电流 / A');xlabel('首次阶跃后的时间 / ms');legend('iq指令','实际iq','Location','best');
 title(sprintf('首次阶跃放大：2%%调节时间 %.2fms',stepSettling));
else
 plot(t,omega*30/pi);grid on;ylabel('rpm');title('转子/轮组转速（非速度闭环）');
end
nexttile;plot(t,hypot(y(:,14),y(:,15)));yline(p.Vdc/sqrt(3),'--');grid on;ylabel('V');title('dq电压幅值与母线约束');
nexttile;plot(t,phase);grid on;ylabel('A');xlabel('时间 / s');title('理想三相电流');legend('ia','ib','ic');
nexttile;plot(t,duty);grid on;ylim([-.05 1.05]);ylabel('占空比');xlabel('时间 / s');title('SVPWM平均占空比（不含开关纹波）');
sgtitle(sprintf('%s | %.1fV | R=%.0fmΩ L=%.0fμH | Kt≈%.5fNm/A | pass=%d', ...
 scenario,p.Vdc,p.Rs*1000,p.Ld*1e6,p.Kt,r.pass));
exportgraphics(f,fullfile(out,'results.png'),'Resolution',140);
if ~showWindows,close(f);end
fprintf('%s: pass=%d, settled iq error=%.6g A, peak torque=%.5g Nm, voltage saturation=%.1f%%, energy residual=%.3g\n', ...
 scenario,r.pass,steadyError,r.peak_torque_Nm,100*r.voltage_saturated_fraction,energyResidual);
end
