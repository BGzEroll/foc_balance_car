function d=calculate_lqr
% 打开本文件，修改下面的参数，点击“运行”，命令窗口显示离散LQR增益K。
% 单位均为SI；角度和角速度使用rad、rad/s。默认值来自机械参数.json。
% 仅修改控制器的设计参数，不修改URDF中3D被控对象的质量和惯量。

%% 可修改：机械参数
p.body_mass = 0.2771;               % 车身固定组质量，kg（含电机定子）
p.wheel_mass = 0.0627;              % 单侧旋转组质量，kg（左右对称）
p.wheel_radius = 0.0325;            % 轮胎半径，m
p.track = 0.1325364;               % 左右轮接触中心间距，m
p.com_height = 0.03652922742030107; % 车身质心距轮轴的高度，m
p.com_forward = -5.7139359606250176e-5; % 车身质心前向偏移，m
p.pitch_inertia = 0.00031184766836104503; % 车身绕质心俯仰惯量，kg*m^2
p.wheel_inertia = 2.996788998409844e-5;   % 单轮组绕驱动轴惯量，kg*m^2
p.yaw_inertia = 0.0012037540146299514;    % 整车原地转向有效惯量，kg*m^2
p.gravity = 9.80665;               % 重力加速度，m/s^2

%% 可修改：采样周期、权重和转矩上限
Ts = 0.005;                        % 控制周期，s
% 状态顺序：[路程 速度 俯仰偏差 俯仰角速度 偏航角 偏航角速度]
Q = diag([100 4 1/deg2rad(8)^2 0.25 1/deg2rad(20)^2 0.2]);
% 输入顺序：[左右轮转矩之和 右轮转矩减左轮转矩]
R = diag([625 625]);
torque_limit_each = 0.10;           % 单轮限幅，N*m；Inf表示不限幅，不参与求K

%% 计算；通常无需修改以下部分
d=solve_balance_lqr(p,Q,R,Ts,torque_limit_each);
if nargout==0
 fprintf('六状态离散LQR，采样周期 Ts = %.6g s\n',Ts);
 fprintf('状态：[s, v, pitch_error, pitch_rate, yaw, yaw_rate]\n');
 fprintf('控制律：[总转矩; 右减左转矩] = -K * (状态 - 指令)\n');
 fprintf('K =\n');disp(d.K);
 fprintf('闭环特征值最大模 = %.9f（小于1表示离散线性闭环稳定）\n',max(abs(eig(d.Ad-d.Bd*d.K))));
 fprintf('静态俯仰平衡角 = %.6f deg\n',rad2deg(d.pitch_trim));
 clear d
end
end
