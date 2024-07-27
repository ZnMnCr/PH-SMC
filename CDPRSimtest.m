clear
close all
clc


% Set Figure default values
set(0,'DefaultTextInterpreter','latex');
set(0,'DefaultLegendInterpreter','latex');
set(0,'DefaultAxesFontSize',11);
set(0,'DefaultLineLineWidth',2.0);
set(0,'DefaultAxesLineWidth',0.5);
set(0,'defaultAxesXGrid','on')
set(0,'defaultAxesYGrid','on')
set(0,'defaultAxesNextPlot','add')

%% Simulation settings

% Simulation step 仿真步长
sim.delta_t = 0.01;
% Simulation length仿真时长
sim.t_end = 20;
pa.m=6000;%动平台质量
pa.g=[0;0;9.8];%重力加速度
%% Define 6DOF CDPR
syms q1 q2 q3 q4 q5 q6 p1 p2 p3 p4 p5 p6 t_sym
q_sym = [q1 q2 q3 q4 q5 q6].';
p_sym = [p1 p2 p3 p4 p5 p6].';


[sys.M,sys.P]=massMatrix(pa.m,pa.g);
sys.V=sys.P;
sys.H = @(q,p) 0.5*p.'*(sys.M(q)\p) + sum(sys.V(q));
sys.D= @(q) diag([0,0,0,0,0,0]);
sys.G = @(q) diag([1;1;1;1;1;1]);
sys.Hdq = matlabFunction(jacobian(sys.H(q_sym,p_sym),q_sym).','vars',[{q_sym}, {p_sym}]);
sys.Hdp =  matlabFunction(jacobian(sys.H(q_sym,p_sym),p_sym).','vars',[{q_sym}, {p_sym}]);
sys.dVdq = matlabFunction(jacobian(sum(sys.V(q_sym)),q_sym).','vars',{q_sym});
dx = @(q,p,u) [zeros(6) eye(6); -eye(6) -sys.D(q)]*[sys.Hdq(q,p); sys.Hdp(q,p)] + [zeros(6); sys.G(q)]*u;
ctrl.T =matlabFunction(manualCholesky(inv(sys.M(q_sym))),'vars',{q_sym});

ctrl.p = @(q,p) ctrl.T(q)'*p;
ctrl.D = @(q) sys.D(q);
ctrl.G = @(q) ctrl.T(q)'*sys.G(q);

ctrl.v=@(t,q,p) [60000;0;0;0;0;0];
%input u from eq.(20)
ctrl.u = @(t,q,p) ctrl.G(q)\(ctrl.T(q)'*sys.dVdq(q) )+ ctrl.v(t,q,p);%额外的输入放在了外面

% Define  closed-loop energy in eq.(24)




%% Run simulation
% Define initial conditions
sim.q0 = [0 0 0 0 0 0].';
sim.p0 = [0 0 0 0 0 0].';
sim.x0 = [sim.q0; sim.p0];

% Comcatinate model with control law
ode = @(t,x) dx(x(1:6),x(7:12),ctrl.u(t,x(1:6),ctrl.p(x(1:6),x(7:12))));
res.x=zeros(3,1);
res.t=zeros(3,1);
% Solve ODE
[res.t,res.x] = ode45(ode,[0:sim.delta_t:sim.t_end],sim.x0,odeset('RelTol',1e-3));
%% Plot output
% Unpack solution vector. Solution is in cannonical coordinates
res.q = res.x(:,1:6);
res.p0 = res.x(:,7:12);


% Compute qualtities of interest
res.H = zeros(length(res.t),1);
res.Hsmc = zeros(length(res.t),1);
res.Hcl = zeros(length(res.t),1);
res.Hp = zeros(length(res.t),1);
res.qd = zeros(length(res.t),6);
res.qe = zeros(length(res.t),6);
res.phi=zeros(length(res.t),6);
for i=1:length(res.t)
    res.H(i)    = sys.H(res.q(i,:).',res.p0(i,:).');
    res.p(i,:)  = (ctrl.T(res.q(i,:).')'*res.p0(i,:).').';
    res.u(i,:)  =ctrl.u(res.t(i,:).',res.q(i,:).',ctrl.p(res.q(i,:).',res.p0(i,:).'));
    
end

% plot energy
fig1 = figure(1)
subplot(2,1,1)
plot(res.t,res.H)
xlabel('time (s)')
ylabel('Open-loop energy')


% plot configuration
fig2 = figure(2)
plot(res.t,res.q)
% 
% plot(res.t,res.qd(:,1:3),'--')
legend('$q_1$','$q_2$','$q_3$','$q_4$','$q_5$','$q_6$')
xlabel('time (s)')
ylabel('Configuration')
title('The responses of q')
grid on

fig4=figure(4)
plot(res.t,res.u)
legend('$q_1$','$q_2$','$q_3$','$q_4$','$q_5$','$q_6$')
title('The responses of u')