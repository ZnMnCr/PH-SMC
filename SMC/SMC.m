clear; clc;

% 系统参数
m1 = 1; m2 = 1; % 关节质量
l1 = 1; l2 = 1; % 关节长度
lc1 = 0.5; lc2 = 0.5; % 质心长度
I1 = 1; I2 = 1; % 转动惯量
g = 9.81; % 重力加速度

% 控制参数
lambda = diag([10, 10]); % 滑模面参数
K = diag([50, 50]);      % 控制增益

% 仿真参数
dt = 0.0001;              % 仿真时间步长
T =5;                   % 仿真总时间
steps = T/dt;            % 仿真步数

% 初始化状态
theta = [0; 0];
theta_dot = [0; 0];

% 期望轨迹

theta_d_hist = [(1/2)*cos((0:dt:T-dt))+sin(2*(0:dt:T-dt)); 2*(1/2)*cos((0:dt:T-dt))+sin(2*(0:dt:T-dt));];
theta_dot_d_hist =  [-(1/2)*sin((0:dt:T-dt))+2*cos(2*(0:dt:T-dt)); -2*(1/2)*sin((0:dt:T-dt))+2*cos(2*(0:dt:T-dt));];
theta_ddot_d_hist =[-(1/2)*cos((0:dt:T-dt))-4*sin(2*(0:dt:T-dt)); -2*(1/2)*cos((0:dt:T-dt))-4*sin(2*(0:dt:T-dt));];
% 存储结果
theta_hist = zeros(2, steps);
theta_dot_hist = zeros(2, steps);
tau_hist = zeros(2, steps);
s_hist = zeros(2, steps);
s_dot_hist = zeros(2, steps);
e_hist = zeros(2,steps);
% 仿真循环
for i = 1:steps
    % 计算期望值
    theta_d = theta_d_hist(:, i);
    theta_dot_d = theta_dot_d_hist(:, i);
    theta_ddot_d = theta_ddot_d_hist(:, i);
    
    % 计算系统矩阵
    M11 = I1 + I2 + m2*l1^2 + 2*m2*l1*lc2*cos(theta(2));
    M12 = I2 + m2*l1*lc2*cos(theta(2));
    M21 = M12;
    M22 = I2;
    M = [M11, M12; M21, M22];
    
    C11 = -m2*l1*lc2*sin(theta(2))*theta_dot(2);
    C12 = -m2*l1*lc2*sin(theta(2))*(theta_dot(1) + theta_dot(2));
    C21 = m2*l1*lc2*sin(theta(2))*theta_dot(1);
    C22 = 0;
    C = [C11, C12; C21, C22];
    
    G1 = (m1*lc1 + m2*l1)*g*cos(theta(1)) + m2*lc2*g*cos(theta(1) + theta(2));
    G2 = m2*lc2*g*cos(theta(1) + theta(2));
    G = [G1; G2];
    
    % 计算控制输入、滑模变量和趋近律
    [tau, s, s_dot] = control_law(theta, theta_dot, theta_d, theta_dot_d, theta_ddot_d, lambda, K, M, C, G);
    
    % 动力学模型 (简化欧拉积分)
    theta_ddot = M \ (tau - C*theta_dot - G);
    
    % 状态更新
    theta_dot = theta_dot + theta_ddot * dt;
    theta = theta + theta_dot * dt;
    
    % 存储历史数据
    theta_hist(:, i) = theta;
    theta_dot_hist(:, i) = theta_dot;
    tau_hist(:, i) = tau;
    s_hist(:, i) = s;  % 记录滑模面 s
    s_dot_hist(:, i) = s_dot;  % 记录趋近律 s_dot
     e_hist(:, i)  = theta - theta_d;  % 角度误差
end

% 绘图
fig=figure(1);

% 关节角度及期望轨迹
subplot(6, 2, 1);
plot(0:dt:T-dt, theta_hist(1, :), 'b', 0:dt:T-dt, theta_hist(2, :), 'r', ...
     0:dt:T-dt, theta_d_hist(1, :), 'b--', 0:dt:T-dt, theta_d_hist(2, :), 'r--');
title('Joint Angles');
legend('$\theta_1$', '$\theta_2$', '$\theta_{d1}$', '$\theta_{d2}$', 'Interpreter', 'latex');

% 关节角速度及期望轨迹
subplot(6, 2, 2);
plot(0:dt:T-dt, theta_dot_hist(1, :), 'b', 0:dt:T-dt, theta_dot_hist(2, :), 'r', ...
     0:dt:T-dt, theta_dot_d_hist(1, :), 'b--', 0:dt:T-dt, theta_dot_d_hist(2, :), 'r--');
title('Joint Velocities');
legend('$\dot{theta}_1$', '$\dot{theta}_2$', '$\dot{theta}_{d1}$', '$\dot{theta}_{d2}$', 'Interpreter', 'latex');

% 控制输入 (力矩)
subplot(6, 2, 3);
plot(0:dt:T-dt, tau_hist(1, :), 0:dt:T-dt, tau_hist(2, :));
title('Control Input (Torque)');
legend('$tau_1$', '$tau_2$', 'Interpreter', 'latex');

% 滑模变量 s
subplot(6, 2, 4);
plot(0:dt:T-dt, s_hist(1, :), 0:dt:T-dt, s_hist(2, :));
title('Sliding Surface (s)');
legend('$s_1$', '$s_2$', 'Interpreter', 'latex');

% 趋近律 s\_dot
subplot(6, 2, 5);
plot(0:dt:T-dt, s_dot_hist(1, :), 0:dt:T-dt, s_dot_hist(2, :));
title('Sliding Mode Approach Law (s_dot)');
legend('$\dot{s}_1$', '$\dot{s}_2$', 'Interpreter', 'latex');
%  error
subplot(6, 2, 6);
plot(0:dt:T-dt, e_hist(1, :), 0:dt:T-dt, e_hist(2, :));
title('system error');
legend('$e_1$', '$e_2$', 'Interpreter', 'latex');

function [tau, s, s_dot] = control_law(theta, theta_dot, theta_d, theta_dot_d, theta_ddot_d, lambda, K, M, C, G)
    % 计算滑模面 s
    e = theta - theta_d;        % 角度误差
    e_dot = theta_dot - theta_dot_d; % 角速度误差
    s = e_dot + lambda * e;%linear SMS
    % beta = 50;
    % p = 3 ;
    % q = 1;
    % s = e_dot + beta .* e.^(p/q); %TSM1
    % s = e + beta .* e_dot.^(p/q); %TSM2
    % 计算趋近律 s_dot
    s_dot = -K * sign(s); 
    % 计算控制律 tau
    tau = M * (theta_ddot_d - lambda * e_dot - K * sign(s)) + C *theta_dot + G;%LSM
    %tau = M*(s_dot-beta.*e_dot.^(p/p-1).*e_dot)+ C * theta_dot + G+s_dot;%TSM1
    %the follow tau has a problem that its dimension does not match theta.
    %tau = M*((s_dot-e_dot)/(beta*(p/q)*e_dot.^(p/q-1)))+ C * theta_dot + G
    
end

