function analyze_smooth_rotation(dataDir,label,amplitude)
% 同时检查原始速度、实际角度轨迹和有限时间位移，区分量化噪声与真实停顿。
if nargin<1 || isempty(dataDir), dataDir=fullfile(fileparts(fileparts(mfilename('fullpath'))),'results','anticogging'); end
if nargin<2, label='track20'; end
if nargin<3, amplitude=.5; end
f=figure('Visible','off','Color','w','Position',[50 50 1500 950]); tiledlayout(2,2);
results=struct([]); k=0;
for m=0:1
    for direction=[1 -1]
        tag='pos'; if direction<0, tag='neg'; end
        path=fullfile(dataDir,sprintf('%s_m%d_%s.csv',label,m,tag));
        if ~isfile(path), continue; end
        r=readtable(path); r=r(r.t>=3,:); t=r.t;
        ref=direction*amplitude; angle=unwrap(r.angle);
        positionError=angle-ref*t; positionError=positionError-mean(positionError);
        dt=median(diff(t)); n20=round(.020/dt); n50=round(.050/dt);
        velocity20=(angle(1+n20:end)-angle(1:end-n20))./(t(1+n20:end)-t(1:end-n20));
        velocity50=(angle(1+n50:end)-angle(1:end-n50))./(t(1+n50:end)-t(1:end-n50));
        pause=abs([0;diff(angle)])<.0001; longest=0; current=0;
        for j=1:numel(pause)
            if pause(j), current=current+dt; else, current=0; end
            longest=max(longest,current);
        end
        k=k+1; results(k).motor=m; results(k).reference_rad_s=ref;
        results(k).mean_speed=(angle(end)-angle(1))/(t(end)-t(1));
        results(k).raw_speed_std=std(r.speed);
        results(k).velocity20_std=std(velocity20); results(k).velocity50_std=std(velocity50);
        results(k).minimum_forward_velocity20=min(direction*velocity20);
        results(k).minimum_forward_velocity50=min(direction*velocity50);
        results(k).position_error_std_deg=std(positionError)*180/pi;
        results(k).position_error_peak_to_peak_deg=range(positionError)*180/pi;
        results(k).longest_encoder_pause_s=longest;
        results(k).feedback_current_rms_A=rms(r.target);
        nexttile; yyaxis left; plot(t(n50+1:end),velocity50,'LineWidth',1); hold on;
        yline(ref,'k:'); ylabel('50 ms 实际位移平均速度 (rad/s)');
        yyaxis right; plot(t,positionError*180/pi,'Color',[.85 .45 .15]); ylabel('角度跟踪误差 (°)');
        xlabel('时间 (s)'); grid on; title(sprintf('M%d，目标 %+.2f rad/s',m,ref));
    end
end
writetable(struct2table(results),fullfile(dataDir,[label '_metrics.csv']));
fid=fopen(fullfile(dataDir,[label '_metrics.json']),'w'); fprintf(fid,'%s',jsonencode(results,PrettyPrint=true)); fclose(fid);
exportgraphics(f,fullfile(dataDir,[label '_motion.png']),'Resolution',160);
disp(struct2table(results));
end

