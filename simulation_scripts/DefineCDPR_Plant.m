function [sys]=DefineCDPR_Plant(m,g)

syms q1 q2 q3 q4 q5 q6 p1 p2 p3 p4 p5 p6 t_sym
q_sym = [q1 q2 q3 q4 q5 q6].';
p_sym = [p1 p2 p3 p4 p5 p6].';

[sys.M,sys.P]=massMatrix(m,g);%计算质量矩阵和势能函数

sys.V=sys.P;
sys.H = @(q,p) 0.5*p.'*(sys.M(q)\p) + sum(sys.V(q));%CDPR的能量函数
%sys.D= @(q) diag([5,2,3,1,2,1]);
sys.D= @(q) diag([0.1,0.1,0.1,0.1,0.1,0.1]);
sys.G = @(q) diag([1;1;1;1;1;1]);
sys.Hdq = matlabFunction(jacobian(sys.H(q_sym,p_sym),q_sym).','vars',[{q_sym}, {p_sym}]);
sys.Hdp =  matlabFunction(jacobian(sys.H(q_sym,p_sym),p_sym).','vars',[{q_sym}, {p_sym}]);
sys.dVdq = matlabFunction(jacobian(sum(sys.V(q_sym)),q_sym).','vars',{q_sym});

% 定义匹配干扰
t_start_disturbance = 3; % 干扰开始的时间
disturbance_amplitude = [1000000;2000000;700000;1000000;0;0]*0.1; % 干扰的幅值
sys.match_distur = @(t) disturbance_amplitude *sin(t)* (t >= t_start_disturbance);
sys.dx = @(q,p,u,t) [zeros(6) eye(6); -eye(6) -sys.D(q)]*[sys.Hdq(q,p); sys.Hdp(q,p)] + [zeros(6); sys.G(q)]*u+ [zeros(6); sys.G(q)]*sys.match_distur(t);