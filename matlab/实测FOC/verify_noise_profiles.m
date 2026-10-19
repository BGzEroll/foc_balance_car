function r=verify_noise_profiles
here=fileparts(mfilename('fullpath'));f=jsondecode(fileread(fullfile(here,'sensor_noise_parameters.json')));rng(8371);rows=struct([]);
fig=figure('Visible','off','Color','w','Position',[40 40 1450 800]);tl=tiledlayout(2,3,'Padding','compact');
for family={'acc','gyro'}
 for profile={'static','10','20'}
  name=[family{1} '_ar_' profile{1}];p=f.(name);state=zeros(12,3);data=zeros(10000,3);
  for k=1:10000,y=-sum(p.coefficients.*state,1)'+p.innovation_chol*randn(3,1);state=[y';state(1:end-1,:)];data(k,:)=y';end
  ratio=var(data(2001:end,:))./p.target_variance(:)';row=struct('profile',name,'variance_ratio',ratio,'max_pole',p.maximum_pole_magnitude,'pass',all(ratio>.5&ratio<1.6)&&p.maximum_pole_magnitude<1);
  if isempty(rows),rows=row;else,rows(end+1)=row;end
  freq=linspace(0,100,1025)';H=zeros(numel(freq),3);for axis=1:3,H(:,axis)=1./(1+exp(-1i*2*pi*freq*(1:12)/200)*p.coefficients(:,axis));end
  C=p.innovation_chol*p.innovation_chol';PSD=2/200*sum(abs(H).^2.*diag(C)',2);
  nexttile;semilogy(p.spectrum_frequency_hz,p.spectrum_total_PSD,'Color',[.4 .4 .4]);hold on;semilogy(freq,PSD,'LineWidth',1.1);grid on;xlim([0 100]);xlabel('Hz');ylabel('三轴PSD和');title(strrep(name,'_','\_'));legend('记录估计','AR12理论谱');
 end
end
title(tl,'实测静止/架空振动与仿真有色扰动：频谱形状对照');exportgraphics(fig,fullfile(here,'noise_profile_validation.png'),'Resolution',140);savefig(fig,fullfile(here,'noise_profile_validation.fig'));close(fig);
r=rows;fid=fopen(fullfile(here,'noise_profile_validation.json'),'w');fprintf(fid,'%s',jsonencode(r,PrettyPrint=true));fclose(fid);disp(r);assert(all([r.pass]));
end
