function fit_cogging(dataDir)
% 两方向静态保持电流拟合；编码器原始单圈角度索引，Iq 保留FOC符号。
if nargin == 0, dataDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results', 'anticogging'); end
n = 4096; order = 120; angles = (0:n-1)'*2*pi/n;
map = zeros(n,2); metrics = struct([]);
f = figure('Visible','off','Color','w','Name','双电机齿槽标定','Position',[100 100 1350 760]);
tiledlayout(2,2);
for m = 0:1
    t = readtable(fullfile(dataDir,sprintf('scan_m%d.csv',m)));
    assert(height(t)==720 && all(isfinite(t.iq)), '扫描数据不完整');
    curves = zeros(n,2); residual = zeros(2,1); coefficients = zeros(2*order+1,2);
    for d = 1:2
        pick = t.direction == (3-2*d);
        x = t.angle(pick); y = t.iq(pick);
        A = basis(x,order); G = basis(angles,order);
        % 四阶频率惩罚抑制高次拟合噪声；直流只参与辨识，不作为补偿。
        penalty = [0 repelem((1:order)/order,2).^4]*5;
        c = (A'*A + diag(penalty)) \ (A'*y);
        residual(d) = sqrt(mean((A*c-y).^2));
        c(1) = 0; curves(:,d) = G*c; coefficients(:,d) = c;
    end
    spatial = coefficients(2:2:end,:) - 1i*coefficients(3:2:end,:);
    amplitude = abs(mean(spatial,2));
    % 仅保留双方向相位一致、且幅值高于拟合噪声的空间谐波。
    keep = real(spatial(:,1).*conj(spatial(:,2))) > 0 & ...
           amplitude > max(0.004, 3*sqrt(mean(residual.^2)/180));
    common = mean(coefficients,2);
    common(2:2:end) = common(2:2:end).*keep;
    common(3:2:end) = common(3:2:end).*keep;
    map(:,m+1) = G*common;
    assert(max(abs(map(:,m+1)))<=6,'补偿电流超过main的6A阈值，拒绝导出');
    metrics(m+1).motor=m;
    metrics(m+1).measured_positions_per_direction=360;
    metrics(m+1).lookup_points=n;
    metrics(m+1).peak_current_A=max(abs(map(:,m+1)));
    metrics(m+1).fit_rms_A=residual';
    metrics(m+1).harmonics=find(keep)';
    metrics(m+1).direction_difference_rms_A=rms(curves(:,1)-curves(:,2));
    nexttile; plot(t.angle(t.direction==1)*180/pi,t.iq(t.direction==1),'.'); hold on;
    plot(t.angle(t.direction==-1)*180/pi,t.iq(t.direction==-1),'.');
    plot(angles*180/pi,map(:,m+1),'k','LineWidth',1.3); grid on;
    xlabel('机械角度 (°)'); ylabel('保持电流 Iq (A)'); title(sprintf('电机 M%d：双向标定',m));
    legend('正向接近','反向接近','去直流补偿','Location','best');
    nexttile; spectrum=abs(fft(map(:,m+1)))*2/n;
    stem(1:100,spectrum(2:101),'.'); grid on; xlabel('每机械圈谐波阶次'); ylabel('Iq 幅值 (A)');
    title(sprintf('M%d 补偿空间频谱',m));
end
motor = repelem((0:1)',n); index = repmat((0:n-1)',2,1); iq=map(:);
writetable(table(motor,index,iq),fullfile(dataDir,'cogging_map.csv'));
save(fullfile(dataDir,'cogging_calibration.mat'),'map','metrics','order');
fid=fopen(fullfile(dataDir,'cogging_map.h'),'w');
fprintf(fid,'#pragma once\nnamespace cogging {\nconstexpr unsigned SIZE = 4096;\ninline float table[2][SIZE] = {\n');
for m=1:2
    fprintf(fid,'{\n');
    for j=1:n
        fprintf(fid,'%.8ff,',map(j,m));
        if mod(j,8)==0, fprintf(fid,'\n'); end
    end
    fprintf(fid,'\n},\n');
end
fprintf(fid,'};\ninline unsigned enabled = 0;\n}\n'); fclose(fid);
fid=fopen(fullfile(dataDir,'calibration_metrics.json'),'w'); fprintf(fid,'%s',jsonencode(metrics,PrettyPrint=true)); fclose(fid);
exportgraphics(f,fullfile(dataDir,'cogging_calibration.png'),'Resolution',160);
disp(metrics);
end
function A = basis(x,k)
A=ones(numel(x),2*k+1);
for h=1:k, A(:,2*h)=cos(h*x); A(:,2*h+1)=sin(h*x); end
end






