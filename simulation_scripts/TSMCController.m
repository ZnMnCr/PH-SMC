%% This script defines the TSMC controller, and its generated energy function.
% Defined matrices of related symbols such as masses or trajectories, 
% passed in via sys and ctrl
% input: sys, ctrl
% output: ctrl
function [ctrl]=TSMCController(sys,ctrl)
syms q1 q2 q3 q4 q5 q6 p1 p2 p3 p4 p5 p6 t_sym k4
q_sym = [q1 q2 q3 q4 q5 q6].';
p_sym = [p1 p2 p3 p4 p5 p6].';

ctrl.T =matlabFunction(manualCholesky(inv(sys.M(q_sym))),'vars',{q_sym});

ctrl.p = @(q,p) ctrl.T(q)'*p;
ctrl.D = @(q) sys.D(q);
ctrl.G = @(q) ctrl.T(q)'*sys.G(q);
% Define target trajectory and derivatives

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
 %K=tril(ones(6));
 alpha=diag([500000;500000;500000;500000;500000;500000])*0.05;
 K=diag([500;500;500;500;500;300])*0.1;
beta = 10000*0.05;
gamma = 3;
delta = 1;
ctrl.phi=@(t,q,p) alpha*ctrl.eq(t,q) + K*ctrl.ep(t,q,p) + beta.*ctrl.eq(t,q).^(gamma/delta);%Here q is a variable to be determined
%phi=@(q,p) K*q+tan(p);
%Take a partial derivative of \phi
dphideq = matlabFunction(jacobian(ctrl.phi(t_sym,q_sym,p_sym),q_sym),'vars',{t_sym,q_sym});
dphidep = matlabFunction(jacobian(ctrl.phi(t_sym,q_sym,p_sym),p_sym),'vars',{p_sym});
%Replace the variable to be determined q with eq(\tilde \q)

%compute the Lambda from eq.(22)
   %   K3 = diag([5;5;5;5;5;5]);
       K3 = diag([0;0;0;0;0;0]);
   %  ctrl.Kd =@(t,q) K3 + 100*(1-gbellmf(ctrl.eq(t,q),[2.5,0.9,0])).*K3;
  ctrl.Kd = @(t,q) K3;
he=sym(0.5*(dphideq(t_sym,q_sym)*(ctrl.T(q_sym)))*dphidep(p_sym)');%这里的系数对系统收敛到滑模面上有影响

Lambda=matlabFunction(2*(he+he'),'vars',[{t_sym,q_sym,p_sym}]);

%Compute the partial derivative of U
normPhi=ctrl.phi(t_sym,q_sym,p_sym);
mu= 0.6;
% p_i = 0.5;
% m_i = 0.5;
% k4 = diag([2;2;2;2;2;2]);
% k4 = -p_i*m_i*k4+p_i.*abs(ctrl.phi(t_sym,q_sym,p_sym));

dUdPhi = matlabFunction(abs(ctrl.phi(t_sym,q_sym,p_sym)).^(1-mu).*((ctrl.phi(t_sym,q_sym,p_sym))/(sqrt(sum(normPhi.^2))))+k4*((ctrl.phi(t_sym,q_sym,p_sym))/(sqrt(sum(normPhi.^2)))),'vars',[{k4},{t_sym},{q_sym},{p_sym}]);
% dUdPhi=matlabFunction((ctrl.phi(t_sym,q_sym,p_sym))/(sqrt(sum(normPhi.^2))),'vars',[{t_sym},{q_sym},{p_sym}]);%base TSMC's energy function  derivative
%Then the feedback controller from eq.(23)
   ctrl.v=@(k4,t,q,p) (-inv(dphidep(p))*Lambda(t,q,p)*dUdPhi(k4,t,q,p)+(ctrl.D(q))*ctrl.ep(t,q,p)-((inv(dphidep(p))*dphideq(t,q))*(ctrl.T(q)))*ctrl.ep(t,q,p));
%   ctrl.v=@(t,q,p) -inv(dphidep(t,q,p))*Lambda(t,q,p)*dUdPhi(t,q,p)+(ctrl.D(q))*ctrl.ep(t,q, p)-((inv(dphidep(t,q,p))*dphidq())*ctrl.T(q))*ctrl.ep(t,q,p));
   % ctrl.v=@(t,q,p) 0;
%input u from eq.(20)
 ctrl.u = @(k4,t,q,p) ctrl.G(q)\(ctrl.D(q)*ctrl.pd(t,q) + ctrl.dpddq(t,q)*(ctrl.T(q)*p) + ctrl.T(q)\ctrl.ddqd(t) + ctrl.T(q)'*sys.dVdq(q) + ctrl.v(k4,t,q,p)-ctrl.Kd(t,q)*ctrl.ep(t,q,p));
% ctrl.u=@(t,q,p) ctrl.G(q)\(-Kd*p);
% Define  closed-loop energy in eq.(24)
ctrl.KE = @(t,q,p) 0.5*sum(ctrl.ep(t,q,p).^2);%kinetic energy
ctrl.U=matlabFunction(sum(abs(normPhi))^(2-mu)+(sqrt(sum(normPhi.^2))),'vars',[{t_sym},{q_sym},{p_sym}]);%potential energy
ctrl.Hd = @(t,q,p) ctrl.KE(t,q,p) + ctrl.U(t,q,p);
