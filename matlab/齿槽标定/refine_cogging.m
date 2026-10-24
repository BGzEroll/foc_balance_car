function refine_cogging(dataDir)
% 利用连续跟踪时的真实Iq重标定，双方向去除运动摩擦，避免静摩擦区间偏差。
if nargin==0, dataDir=fullfile(fileparts(fileparts(mfilename('fullpath'))),'results','anticogging'); end
old=load(fullfile(dataDir,'cogging_calibration.mat'),'map');
n=4096; order=120; angle=(0:n-1)'*2*pi/n; G=basis(angle,order);
map=zeros(n,2); metrics=struct([]);
f=figure('Visible','off','Color','w','Position',[60 60 1400 850]); tiledlayout(2,2);
for m=0:1
    coefficients=zeros(2*order+1,2); residual=zeros(1,2); dc=zeros(1,2);
    for d=1:2
        tag='pos'; if d==2, tag='neg'; end
        r=readtable(fullfile(dataDir,sprintf('track20_m%d_%s.csv',m,tag)));
        r=r(r.t>=3,:);
        A=basis(r.angle,order); y=r.iq;
        penalty=[0 repelem((1:order)/order,2).^4]*10;
        c=(A'*A+diag(penalty))\(A'*y);
        residual(d)=rms(A*c-y); dc(d)=c(1); c(1)=0; coefficients(:,d)=c;
    end
    z=coefficients(2:2:end,:)-1i*coefficients(3:2:end,:);
    amplitude=abs(mean(z,2));
    coherence=real(z(:,1).*conj(z(:,2)))./(abs(z(:,1)).*abs(z(:,2))+eps);
    keep=amplitude>.002 & coherence>.5;
    common=mean(coefficients,2); common(2:2:end)=common(2:2:end).*keep; common(3:2:end)=common(3:2:end).*keep;
    map(:,m+1)=G*common;
    metrics(m+1).motor=m; metrics(m+1).lookup_points=n;
    metrics(m+1).direction_friction_current_A=dc; metrics(m+1).fit_residual_A=residual;
    metrics(m+1).map_change_rms_A=rms(map(:,m+1)-old.map(:,m+1));
    metrics(m+1).peak_current_A=max(abs(map(:,m+1))); metrics(m+1).harmonics=find(keep)';
    nexttile; plot(angle*180/pi,old.map(:,m+1)); hold on; plot(angle*180/pi,map(:,m+1)); grid on;
    xlabel('机械角度 (°)'); ylabel('补偿 Iq (A)'); title(sprintf('M%d 运动数据重标定',m)); legend('静态标定','匀速数据拟合');
    nexttile; plot(angle*180/pi,map(:,m+1)-old.map(:,m+1)); grid on;
    xlabel('机械角度 (°)'); ylabel('修正 Iq (A)'); title(sprintf('M%d 残余位置相关电流',m));
end
assert(max(abs(map),[],'all')<6,'超过main电流上限');
motor=repelem((0:1)',n); index=repmat((0:n-1)',2,1); iq=map(:);
writetable(table(motor,index,iq),fullfile(dataDir,'dynamic_cogging_map.csv'));
save(fullfile(dataDir,'dynamic_cogging.mat'),'map','metrics','order');
fid=fopen(fullfile(dataDir,'dynamic_cogging_map.h'),'w'); fprintf(fid,'#pragma once\nnamespace cogging {\nconstexpr unsigned SIZE = 4096;\ninline float table[2][SIZE] = {\n');
for m=1:2
    fprintf(fid,'{\n');
    for j=1:n
        fprintf(fid,'%.8ff,',map(j,m)); if mod(j,8)==0, fprintf(fid,'\n'); end
    end
    fprintf(fid,'\n},\n');
end
fprintf(fid,'};\ninline unsigned enabled = 0;\n}\n'); fclose(fid);
fid=fopen(fullfile(dataDir,'dynamic_cogging_metrics.json'),'w'); fprintf(fid,'%s',jsonencode(metrics,PrettyPrint=true)); fclose(fid);
exportgraphics(f,fullfile(dataDir,'dynamic_cogging.png'),'Resolution',160);
disp(metrics);
end
function A=basis(x,k)
A=ones(numel(x),2*k+1); for h=1:k, A(:,2*h)=cos(h*x); A(:,2*h+1)=sin(h*x); end
end

