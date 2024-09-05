% 2DOF 机械臂快速终端滑模控制器示例

clear; clc; close all;

%% 系统参数定义
% 链节长度（米）
l1 = 1;
l2 = 1;

% 链节质量（千克）
m1 = 1;
m2 = 1;

% 重力加速度（米/秒²）
g = 9.81;

% 转动惯量（kg·m²）
I1 = (1/12)*m1*l1^2;
I2 = (1/12)*m2*l2^2;

%% 控制参数
% 快速终端滑模参数
lambda1 = 2;
lambda2 = 1;
alpha1 = 1.5;
alpha2 = 0.5;

% 控制增益
K_p = 10 * eye(2);
K_d = 5 * eye(2);
eta = 5; % 等速趋近律参数

%% 期望轨迹定义（例如，简谐运动）
q_d = @(t) [pi/4 * sin(pi*t/5); pi/6 * sin(pi*t/5)];
qd_dot = @(t) [pi/4 * (pi/5) * cos(pi*t/5); pi/6 * (pi/5) * cos(pi*t/5)];
qd_ddot = @(t) [-pi^2/20 * sin(pi*t/5); -pi^2/30 * sin(pi*t/5)];

%% 动力学模型定义
% 状态空间方程
% 状态向量 x = [q1; q2; q1_dot; q2_dot]
% dx/dt = [q_dot; q_ddot]

% 定义匿名函数用于计算 M(q), C(q, q_dot), G(q)
M = @(q) [I1 + I2 + m2*l1^2 + 2*m2*l1*l2*cos(q(2)), I2 + m2*l1*l2*cos(q(2));
         I2 + m2*l1*l2*cos(q(2)), I2];

C = @(q, q_dot) [-2*m2*l1*l2*sin(q(2))*q_dot(2), -m2*l1*l2*sin(q(2))*q_dot(2);
                m2*l1*l2*sin(q(2))*q_dot(1), 0];

G = @(q) [(m1*l1/2 + m2*l1)*g*cos(q(1)) + m2*l2*g*cos(q(1) + q(2));
         m2*l2*g*cos(q(1) + q(2))];

%% 控制器定义
control_law = @(x, t) controller(x, t, q_d, qd_dot, qd_ddot, M, C, G, ...
    lambda1, lambda2, alpha1, alpha2, K_p, K_d, eta);

%% 初始条件
x0 = [0; 0; 0; 0]; % 初始关节角度和角速度

%% 仿真设置
t_span = [0 10]; % 仿真时间 0 到 10 秒

%% 使用 ode45 进行仿真
[t, x] = ode45(@(t, x) dynamics(t, x, control_law), t_span, x0);

%% 提取结果
q1 = x(:,1);
q2 = x(:,2);
q1_dot = x(:,3);
q2_dot = x(:,4);

% 使用 q_d 生成的期望值
qd_vals = arrayfun(q_d, t, 'UniformOutput', false);
qd_vals = cell2mat(qd_vals');  % 转换为 2xN 的矩阵

% 提取期望关节角度
qd1 = qd_vals(1, :);
qd2 = qd_vals(2, :);

% 在绘图时，使用 qd1 和 qd2
subplot(2,1,1);
plot(t, q1, 'b', t, qd1, 'r--', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('关节角度 q1 (rad)');
legend('实际 q1', '期望 q1');
title('关节角度 q1 跟踪');
grid on;

subplot(2,1,2);
plot(t, q2, 'b', t, qd2, 'r--', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('关节角度 q2 (rad)');
legend('实际 q2', '期望 q2');
title('关节角度 q2 跟踪');
grid on;

%% 控制器函数
function tau = controller(x, t, q_d, qd_dot, qd_ddot, M_func, C_func, G_func, ...
    lambda1, lambda2, alpha1, alpha2, K_p, K_d, eta)

    % 状态变量
    q = x(1:2);
    q_dot = x(3:4);
    
    % 期望轨迹及其导数
    qd = q_d(t);
    qd_dot_val = qd_dot(t);
    qd_ddot_val = qd_ddot(t);
    
    % 误差
    e = qd - q;
    e_dot = qd_dot_val - q_dot;
    
    % 滑模面 s
    s = e + lambda1 * abs(e).^alpha1 .* sign(e) + lambda2 * abs(e).^alpha2 .* sign(e);
    
    % 控制律
    % u = M(q)*(qd_ddot + K_d * s) + C(q, q_dot)*q_dot + G(q) - K_p * sign(s)
    M_val = M_func(q);
    C_val = C_func(q, q_dot);
    G_val = G_func(q);
    
    % 计算控制输入
    tau = M_val * (qd_ddot_val + K_d * s) + C_val * q_dot + G_val - K_p * sign(s);
    
end

%% 动力学函数
function dxdt = dynamics(t, x, control_law)

    % 控制输入
    tau = control_law(x, t);
    
    % 系统参数（与主脚本中一致）
    l1 = 1; l2 = 1;
    m1 = 1; m2 = 1;
    g = 9.81;
    I1 = (1/12)*m1*l1^2;
    I2 = (1/12)*m2*l2^2;
    
    % 状态变量
    q = x(1:2);
    q_dot = x(3:4);
    
    % 定义动力学矩阵
    M = [I1 + I2 + m2*l1^2 + 2*m2*l1*l2*cos(q(2)), I2 + m2*l1*l2*cos(q(2));
         I2 + m2*l1*l2*cos(q(2)), I2];
    
    C = [-2*m2*l1*l2*sin(q(2))*q_dot(2), -m2*l1*l2*sin(q(2))*q_dot(2);
         m2*l1*l2*sin(q(2))*q_dot(1), 0];
     
    G = [(m1*l1/2 + m2*l1)*g*cos(q(1)) + m2*l2*g*cos(q(1) + q(2));
         m2*l2*g*cos(q(1) + q(2))];
    
    % 计算 q_ddot
    q_ddot = M \ (tau - C*q_dot - G);
    
    % 状态导数
    dxdt = [q_dot; q_ddot];
    
end
