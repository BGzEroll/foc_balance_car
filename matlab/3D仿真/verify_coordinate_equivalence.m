function difference=verify_coordinate_equivalence
% 回放上版真实3D测量，验证坐标换基不改变控制律；旧函数仅保留在.cache。
paths=balance_setup;addpath(fullfile(paths.matlab,'.cache'));
file=fullfile(paths.results,'suite','usb_forward','slope','slope_hold_pid_current','simulation.mat');
data=load(file,'o');ts=data.o.raw3d;v=squeeze(ts.Data);if size(v,1)~=numel(ts.Time),v=v';end
[times,index]=unique(ts.Time,'stable');v=v(index,:);at=(0:.005:34)';v=interp1(times,v,at);v(:,1)=at;
new=v(:,[1 4 5 2 3 6 7 8 10 9 11 13 12 14]);new(:,[2 3 9 12])=-new(:,[2 3 9 12]);
cfg=balance_control_config('pid_current',9,[],1,'slope');
clear balance_controller_previous balance_suite_controller
oldOut=zeros(numel(at),25);newOut=oldOut;
for k=1:numel(at)
 oldOut(k,:)=balance_controller_previous(v(k,:)',cfg)';
 newOut(k,:)=balance_suite_controller(new(k,:)',cfg)';
end
newOut(:,15:17)=[newOut(:,16) -newOut(:,15) newOut(:,17)];
newOut(:,18:20)=[newOut(:,19) -newOut(:,18) newOut(:,20)];
difference=max(abs(oldOut-newOut),[],'all');assert(difference<1e-10);
assert(max(abs(balance_control_coordinates(new)-v),[],'all')<1e-12);
clear balance_suite_controller
cfg=balance_control_config('pid_current',5);raw=[2.05;zeros(6,1);1;zeros(6,1)];out=balance_suite_controller(raw,cfg);
assert(abs(out(15))<1e-12&&abs(out(16)-.4)<1e-12); % 推扰沿新世界+Y
fprintf('COORDINATE_EQUIVALENCE_PASSED: max difference=%g\n',difference);
end
