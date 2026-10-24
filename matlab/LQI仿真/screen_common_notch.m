function screen_common_notch
% Screen frequency-shaped common-current feedback. Not an identified flexible plant.
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));out=fullfile(root,'esp32','lqi_firmware','results','jitter_refine');
s=jsondecode(fileread(fullfile(root,'esp32','lqi_firmware','selected_lqi.json')));d=design_lqi;K=d.Kaug.*s.scale(:)';
fc=12.5;w=2*pi*fc*d.Ts;pool=struct([]);
for mode=0:1
for r=[.92 .95 .97 .98 .985]
 g=(1-2*r*cos(w)+r*r)/(2-2*cos(w));b=g*[1 -2*cos(w) 1];a=[1 -2*r*cos(w) r*r];
 for mix=.1:.1:1
  radius=[];
  for kt=[.7 1 1.3]
   for delay=0:2
    for tau=[0 .0048]
     n=20+2*delay;F=zeros(n);
     for j=1:n,e=zeros(n,1);e(j)=1;F(:,j)=step(e,d,K,kt,delay,tau,mix,b,a,mode);end
     radius(end+1)=max(abs(eig(F))); %#ok<AGROW>
    end
   end
  end
  if max(radius)>=1,break;end % Every traversed blend must be stable, not just the endpoint.
  f=[0 1 2 3 5 10 11 12 12.5 13 14 20];z=exp(-1i*2*pi*f*d.Ts);
  H=(b(1)+b(2)*z+b(3)*z.^2)./(a(1)+a(2)*z+a(3)*z.^2);H=1+mix*(H-1);
  item=struct('mode',mode,'frequency_Hz',fc,'pole_radius',r,'mix',mix,'b',b,'a',a,...
   'max_closed_loop_radius',max(radius),'response_f_Hz',f,'magnitude',abs(H),'phase_deg',angle(H)*180/pi,...
   'score',mean(abs(H(8:10)))+.1*mode);
  if isempty(pool),pool=item;else,pool(end+1)=item;end %#ok<AGROW>
 end
end
end
assert(~isempty(pool),'No stable notch candidates');[~,idx]=min([pool.score]);report=pool(idx);
report.scope='Mode0 common effort; mode1 common pitch-rate contribution only. CAD/Kt and delay hypotheses; real A/B required.';
assert(report.mode==0,'Firmware candidate implements common-effort placement only');
trial=d;trial.Kaug=K;trial.K=K(:,1:6);trial.Ki=K(:,7:8);trial.current_limit_each=6;
cases={};
for scenario={'recover','push','bias','noise'}
 options=struct('duration',12,'current_lag_s',.003,'delay_cycles',2,...
  'Kt_scale',.7,'common_notch',report);
 if strcmp(scenario{1},'noise'),options.noise_pitch=.002;options.noise_rate=.03;end
 result=simulate_lqi(trial,scenario{1},options);cases{end+1}=result.report; %#ok<AGROW>
end
report.nonlinear_cases=cases;assert(all(cellfun(@(c)c.pass,cases)),'Notch failed nonlinear screen');
save(fullfile(out,'notch_screen.mat'),'pool','d','K');
fid=fopen(fullfile(out,'notch_design.json'),'w');fprintf(fid,'%s',jsonencode(report,PrettyPrint=true));fclose(fid);
fprintf('NOTCH_STABILITY_PASS mode=%d r=%.3f mix=%.2f max_radius=%.7f DC=%.6f f3_mag=%.4f f3_phase=%.2fdeg\n',report.mode,report.pole_radius,report.mix,report.max_closed_loop_radius,report.magnitude(1),report.magnitude(4),report.phase_deg(4));
end
function next=step(z,d,K,kt,delay,tau,mix,b,a,mode)
x=z(1:6);xi=z(7:8);m=z(9:14);iq=z(15:16);h=z(17:20);
t=[0 .003 tau tau tau tau]';alpha=zeros(6,1);use=t>0;alpha(use)=exp(-d.Ts./t(use));m=alpha.*m+(1-alpha).*x;
u=-K*[m;xi];if mode==0,raw=u(1);else,raw=-K(1,4)*m(4);end
filtered=b(1)*raw+b(2)*h(1)+b(3)*h(2)-a(2)*h(3)-a(3)*h(4);
h=[raw;h(1);filtered;h(3)];u(1)=u(1)+mix*(filtered-raw);xi=xi+d.Ts*d.C*m;
if delay,q=reshape(z(21:end),2,delay);target=q(:,1);q=[q(:,2:end),u];else,target=u;q=[];end
iq=target+(iq-target)*exp(-d.Ts/.003);x=d.Ad*x+d.Bd*(kt*iq);next=[x;xi;m;iq;h;q(:)];
end
