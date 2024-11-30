function [T_optimal,W]=TensionDistribution(J,W)
    %Cable force distribution by quadratic programming with 
    % the constraint that the cable force is greater than 0
    %input；CDPR structural matrix J，End force W
    %output；Optimal tension T_optimal，and the corresponding end forces W

% 绳索数量
num_cables = size(J, 2);


% 定义二次规划参数
H =  eye(num_cables);            % 二次项矩阵 (用于最小化偏差平方和)
f = [];  % 线性项，用于将拉力引导到接近 T_avg

% 不等式约束条件：T >= 0
G = -eye(num_cables);
h = zeros(num_cables, 1);
Aeq = J;
beq = W;
% 优化选项设置
options = optimoptions('quadprog', 'Display', 'off');

% 求解二次规划
[T_optimal, fval, exitflag, output] = quadprog(H, f, G, h,Aeq,beq, [], [], [], options);

W = J*T_optimal;