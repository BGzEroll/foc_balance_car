function summary=analyze_combo_trials
here=fileparts(mfilename('fullpath'));root=fileparts(fileparts(here));folder=fullfile(root,'esp32/lqi_firmware/results/identification');out=fullfile(here,'results/combined_motion');if ~isfolder(out),mkdir(out);end
files=dir(fullfile(folder,'combo_*/report.json'));summary=struct([]);traces={};
for n=1:numel(files)
 r=jsondecode(fileread(fullfile(files(n).folder,'report.json')));if isfield(r,'excluded_from_identification') && r.excluded_from_identification,continue;end
 s=readtable(fullfile(files(n).folder,'samples.csv'));s=s(s.sample_us>=r.start_us,:);y=readtable(fullfile(files(n).folder,'yaw.csv'));
 travel=.0325*(diff(s.angle_b)-diff(s.angle_a))/2+.0325*diff(s.pitch);heading=(s.yaw(1:end-1)+s.yaw(2:end))/2-s.yaw(1);xy=[0,0;cumsum([travel.*cos(heading),travel.*sin(heading)],1)];
 target=interp1(y.time_us,y.target,s.sample_us,'previous','extrap');q=(y.time_us>=r.start_us & y.phase<4);active=s.sample_us<=r.start_us+max(16,4*(r.move_seconds+2))*1e6;
 m=struct('label',string(files(n).folder),'position_command_m',r.amplitude,'yaw_command_deg',r.yaw_degrees,'move_s',r.move_seconds,'peak_planar_m',max(vecnorm(xy,2,2)),'final_xy_m',xy(end,:),'peak_pitch_deg',max(abs(s.pitch))*180/pi,'peak_each_command_A',max(abs([s.command_a;s.command_b])),'yaw_tracking_rms_deg',rms((s.yaw(active)-target(active))*180/pi),'aborted',any(y.phase==5|y.phase==6),'fault',max(s.fault));
 summary=[summary;m];traces{end+1}=struct('s',s,'xy',xy,'target',target,'name',files(n).folder); %#ok<AGROW>
end
fid=fopen(fullfile(out,'summary.json'),'w');fprintf(fid,'%s',jsonencode(summary,PrettyPrint=true));fclose(fid);
draw=find(cellfun(@(x)contains(x.name,'combo_fixed_'),traces));labels=cell(1,numel(draw));
for n=1:numel(draw),m=summary(draw(n));labels{n}=sprintf('%g cm + %g° / %g s',m.position_command_m*100,m.yaw_command_deg,m.move_s);end
f=figure('Visible','off','Color','w','Position',[80 80 1100 800]);tiledlayout(2,2);
nexttile;hold on;for n=draw,plot(traces{n}.xy(:,1)*100,traces{n}.xy(:,2)*100);end;axis equal;grid on;xlabel('里程计前后 (cm)');ylabel('里程计左右 (cm)');title('各次起点对齐；无外部位置真值');legend(labels,'Location','best');
nexttile;hold on;for n=draw,s=traces{n}.s;plot((s.sample_us-s.sample_us(1))*1e-6,s.pitch*180/pi);end;grid on;xlabel('时间 (s)');ylabel('俯仰 (度)');
nexttile;hold on;for n=draw,s=traces{n}.s;plot((s.sample_us-s.sample_us(1))*1e-6,s.distance*100);end;grid on;xlabel('时间 (s)');ylabel('累计前后位移 (cm)');
nexttile;bar([summary(draw).peak_each_command_A]);grid on;ylabel('单轮指令电流峰值 (A)');xticks(1:numel(draw));xticklabels(labels);xtickangle(15);
sgtitle('前后运动＋转向组合验证；疑似碰椅子的首组已排除');exportgraphics(f,fullfile(out,'combined_motion.png'),'Resolution',140);savefig(f,fullfile(out,'combined_motion.fig'));close(f);disp(summary);
end
