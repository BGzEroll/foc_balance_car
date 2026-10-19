function reports=validate_measured_foc
% Replay held-out real shaft motion through the electrical model, not torque-driven fitting.
p=measured_foc_parameters;r=fileparts(mfilename('fullpath'));out=fullfile(r,'validation');if ~isfolder(out),mkdir(out);end
names={'base_d030_r1','mid_d030_repeat_r1','base_q0p2'};reports=struct([]);
f=figure('Visible','off','Color','w','Position',[40 40 1400 1000]);tl=tiledlayout(3,2,'Padding','compact');
for k=1:numel(names)
 name=names{k};T=readtable(fullfile(p.data,[name '.csv']));J=jsondecode(fileread(fullfile(p.data,[name '.json'])));tt=(T.us-J.meta.profile_start)*1e-6;
 cfg=p.cfg;cfg(4)=J.meta.kp;cfg(5)=J.meta.ki;cfg(10)=J.setup.bus_ratio;cfg(11)=J.setup.voltage_cap;
 ts=(0:p.Ts:J.meta.duration_ms*.001)';q=zeros(numel(ts),2);w=q;ref=zeros(2,2,numel(ts));
 for m=1:2
  I=T.motor==m;qt=J.setup.direction(m)*T.angle(I)-J.setup.zero_offset(m)/7;
  if J.meta.mode==3,q(:,m)=qt(1);else,w(:,m)=interp1(tt(I),T.speed(I),ts,'linear','extrap');q(:,m)=qt(1)+cumtrapz(ts,w(:,m));end
 end
 x=measured_foc_initial(q(1,:));Y=zeros(numel(ts),28);
 for n=1:numel(ts)
  t=ts(n);rr=zeros(2,2);
  if J.meta.mode==3
   amp=max(T.id_ref);d=0;if t>=.05&&t<.10,d=.5*amp;elseif t>=.10&&t<.20,d=amp;elseif t>=.20&&t<.30,d=.5*amp;end
   rr(1,:)=d;
  else
   amp=max(T.iq_ref);u=0;if (t>=.04&&t<.06)||(t>=.16&&t<.18),u=amp;elseif (t>=.10&&t<.12)||(t>=.18&&t<.20),u=-amp;end
   rr(2,:)=u;
  end
  [x,y]=measured_foc_step(x,rr,q(n,:),w(n,:),n-1,cfg);Y(n,:)=y(:)';
 end
 for m=1:2
  I=T.motor==m;idx=(m-1)*14;currentColumn=4;dataCurrent=T.id(I);target=T.id_ref(I);
  if J.meta.mode==5,currentColumn=5;dataCurrent=T.iq(I);target=T.iq_ref(I);end
  simAt=interp1(ts,Y(:,idx+currentColumn),tt(I));
  mask=tt(I)>.14&tt(I)<.19;if J.meta.mode==5,mask=abs(target)>0;end
  row=struct('case_name',name,'motor',m,'measured_mean_A',mean(dataCurrent(mask)), ...
   'simulated_mean_A',mean(simAt(mask)),'measured_std_A',std(dataCurrent(mask)), ...
   'simulated_std_A',std(simAt(mask)),'waveform_rmse_A',sqrt(mean((simAt-dataCurrent).^2)), ...
   'validation_type','electrical replay with recorded shaft speed; independent sensor noise realization', ...
   'pass',all(isfinite(Y),'all')&&max(abs(Y(:,[2 3 16 17])),[],'all')<2&&sqrt(mean((simAt-dataCurrent).^2))<.05&&abs(mean(simAt(mask))-mean(dataCurrent(mask)))<.015);
  if J.meta.mode==3,row.pass=row.pass&&std(simAt(mask))/std(dataCurrent(mask))>.5&&std(simAt(mask))/std(dataCurrent(mask))<1.6;end
  if isempty(reports),reports=row;else,reports(end+1)=row;end
  nexttile((k-1)*2+m);plot(tt(I)*1e3,dataCurrent,'Color',[.65 .65 .65]);hold on;plot(ts*1e3,Y(:,idx+currentColumn),'Color',[.2 .5 .85]);plot(tt(I)*1e3,target,'k--');grid on;legend('实测','模型测量值','目标');xlabel('时间 / ms');ylabel('A');title(sprintf('%s / M%d',strrep(name,'_','\_'),m));
 end
 writetable(array2table([ts Y]),fullfile(out,[name '_model.csv']));
end
title(tl,'实测电流环与平均电气模型对照；噪声实现不同，不按单点逐一拟合');exportgraphics(f,fullfile(out,'electrical_replay.png'),'Resolution',140);savefig(f,fullfile(out,'electrical_replay.fig'));close(f);
fid=fopen(fullfile(out,'validation.json'),'w');fprintf(fid,'%s',jsonencode(reports,PrettyPrint=true));fclose(fid);disp(reports);assert(all([reports.pass]));
end
