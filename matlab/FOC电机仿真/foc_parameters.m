function p=foc_parameters
% ZD2808第一版电气模型；本文件集中修改参数，单位SI。
% 原作者工程只读核对：7极对、SVPWM、6A限制；不复制或编译作者代码。
p.pole_pairs=7;
p.KV=700;                          % 商品标称rpm/V，未实测
p.line_resistance=0.180;            % 用户两端子测量，第三根悬空，ohm
p.line_inductance=48e-6;            % 同上，H；测试频率、转子位置待确认
p.Rs=p.line_resistance/2;           % 等效星形每相电阻；DCR口径待确认
p.Ld=p.line_inductance/2;           % 平衡、低凸极性等效模型，初步估计
p.Lq=p.Ld;
p.Vdc=7.4;                         % 两节18650串联的仿真工作点，可改8.4
p.Imax=6;                          % iq指令上限，来自作者配置，非额定电流
p.Ts=1/25000;                      % 仿真电流环40us；不代表作者实测频率
p.bandwidth_hz=500;                % 初版电流环设计带宽
p.Kpd=p.Ld*2*pi*p.bandwidth_hz;
p.Kpq=p.Lq*2*pi*p.bandwidth_hz;
p.Ki=p.Rs*2*pi*p.bandwidth_hz;
% 采用SimpleFOC文档的BLDC KV到相峰值反电势换算约定。
% ke_phase_peak=60/(2*pi*sqrt(3)*KV)，lambda=ke/p，Kt=1.5*p*lambda。
% 属于标称KV推算，并非实测磁链/转矩常数。
p.ke_phase_peak=60/(2*pi*sqrt(3)*p.KV);
p.flux=p.ke_phase_peak/p.pole_pairs;
p.Kt=1.5*p.pole_pairs*p.flux;       % Nm/A，幅值不变Park变换下iq
p.J=2.996788998409844e-5;           % 独立电机台架采用现有单轮组惯量
p.B=1e-5;                         % Nm/(rad/s)，未辨识的教学假设
p.integration_substeps=10;         % 电气RK4步长4us
assert(all(isfinite([p.Rs p.Ld p.Lq p.Vdc p.Imax p.Ts p.J p.B p.flux])));
assert(all([p.Rs p.Ld p.Lq p.Vdc p.Imax p.Ts p.J p.flux]>0)&&p.B>=0);
assert(p.integration_substeps>=1&&p.integration_substeps==round(p.integration_substeps));
% cfg=[p Rs Ld Lq flux J B Vdc Ts Kpd Kpq Ki Imax substeps scenario]。
p.config=[p.pole_pairs p.Rs p.Ld p.Lq p.flux p.J p.B p.Vdc p.Ts ...
 p.Kpd p.Kpq p.Ki p.Imax p.integration_substeps 1];
end
