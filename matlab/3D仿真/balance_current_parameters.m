function p=balance_current_parameters
% PID电流接口的初始电机假设，不是根据原作者PID辨识出的Kt。
% 与独立FOC模型一致：幅值不变Clarke/Park、iq为相电流峰值口径。
p.KV=700;
p.Kt_nominal=1.5*60/(2*pi*sqrt(3)*p.KV); % Nm/A，标称KV近似
p.current_limit_each=6; % A，沿用作者配置的数值；不是已验证额定值
p.ideal_current_tracking=true;
assert(p.Kt_nominal>0&&isfinite(p.Kt_nominal));
assert(p.current_limit_each>0&&isfinite(p.current_limit_each));
end
