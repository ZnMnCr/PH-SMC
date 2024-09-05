clc
clear
syms q1 q2 q3 q4 q5 q6 p1 p2 p3 p4 p5 p6 t_sym
q_sym = [q1 q2 q3 q4 q5 q6].';
p_sym = [p1 p2 p3 p4 p5 p6].';

addpath('./simulation_scripts');
%% Define 6DOF CDPR plant
pa.m=6000;%动平台质量
pa.g=[0;0;9.8];%重力加速度
[sys]=DefineDisturbance();
[sys]=DefineCDPR_Plant(pa.m,pa.g,sys);

ctrl.T =matlabFunction(manualCholesky(inv(sys.M(q_sym))),'vars',{q_sym});

ctrl.p = @(q,p) ctrl.T(q)'*p;
ctrl.qd = @(t) [10*(1/2)*cos(t)+5*(1/2)*sin(2*t); 10*(1/2)*cos(t);10*(1/2)*cos(t)+5*(1/2)*sin(3*t);10*(1/2)*cos(t)+5*(1/2)*sin(2*t); 0;0];
ctrl.dqd = @(t) [-10*(1/2)*sin(t)+10*(1/2)*cos(2*t); -10*(1/2)*sin(t);-10*(1/2)*sin(t)+5*0.5*3*cos(3*t);-10*(1/2)*sin(t)+10*(1/2)*cos(2*t);0;0];
ctrl.ddqd = @(t) [-10*(1/2)*cos(t)-20*(1/2)*sin(2*t); -10*(1/2)*cos(t);-10*(1/2)*cos(t)-5*0.5*3*3*sin(3*t);-10*(1/2)*cos(t)-20*(1/2)*sin(2*t);0;0];

% Compute the target momentum from (13)
%期望动量的坐标变换
ctrl.pd = @(t,q) ctrl.T(q)\ctrl.dqd(t);

% Compute the error coordinates on q, p from (11), (15)
%速度和动量误差
ctrl.ep = @(t,q,p) p - ctrl.pd(t,q);
ctrl.eq = @(t,q) q - ctrl.qd(t);
% Compute the gradient of the reference momentum with respect to
% configuration as per (16)
%求期望动量的梯度
ctrl.dpddq = matlabFunction(jacobian(ctrl.pd(t_sym,q_sym),q_sym),'vars',[{t_sym}, {q_sym}]);
%% Define Passivity-based sliding mode controller
%VI. NUMERICAL EXAMPLE Case1 and K
 %K=tril(ones(6));S

 K1=diag([300;300;300;125;300;300]);
K2=diag([300;300;300;125;300;300]);
ctrl.alpha1=2;
ctrl.alpha2=0.5;

 %ctrl.phi=@(t,q,p) (K*ctrl.eq(t,q)+ctrl.ep(t,q,p));%Here q is a variable to be determined
ctrl.phi = @(t,q,p) ctrl.ep(t,q,p) +K1*(abs(ctrl.eq(t,q)).^ctrl.alpha1).*(ctrl.eq(t,q)/sqrt(sum(ctrl.eq(t,q).^2)))...
    +K2*(abs(ctrl.eq(t,q)).^ctrl.alpha2).*(ctrl.eq(t,q)/sqrt(sum(ctrl.eq(t,q).^2)));
ctrl.phi(1,[1;1;1;1;1;1],[1;1;1;1;1;1]);
dphideq=matlabFunction(jacobian(ctrl.phi(t_sym,q_sym,p_sym),q_sym),'vars',{t_sym,q_sym});
dphidep=matlabFunction(jacobian(ctrl.phi(t_sym,q_sym,p_sym),p_sym),'vars',{p_sym});
%compute the Lambda from eq.(22)
he=sym(0.5*(dphideq(t_sym,q_sym)*ctrl.T(q_sym))*dphidep(p_sym)');%这里的系数对系统收敛到滑模面上有影响
%he=100000*0.5*(dphidq(q_sym)*ctrl.T(q_sym))*dphidep(t_sym,q_sym,p_sym)';
Lambda=matlabFunction(2*(he+he'),'vars',[{t_sym,q_sym,p_sym}]);
%Compute the partial derivative of U
normPhi=ctrl.phi(t_sym,q_sym,p_sym);
dUdPhi=matlabFunction((ctrl.phi(t_sym,q_sym,p_sym))/(sqrt(sum(normPhi.^2))),'vars',[{t_sym},{q_sym},{p_sym}]);
%Then the feedback controller from eq.(23)
ctrl.v=@(t,q,p) (-inv(dphidep(p))*Lambda(t,q,p)*dUdPhi(t,q,p)+(ctrl.D(q))*ctrl.ep(t,q,p)-((inv(dphidep(p))*dphideq(t,q))*ctrl.T(q))*ctrl.ep(t,q,p));