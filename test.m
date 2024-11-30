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
sim.t_end = 5;
% Define initial conditions
sim.q0 = [0 -0.2].';
sim.p0 = [0 0].';
sim.x0 = [sim.q0; sim.p0];
%% Define 2DOF manipulator model
% symbolic variables
syms q1 q2 p1 p2 t_sym
q_sym = [q1 q2].';
p_sym = [p1 p2].';

% Define kinematic parameters
sys.l1 = 1;
sys.l2 = 1;
sys.G = @(q) [1 0; 0 1];

% Define inertial parameters
sys.m1 = 1;
sys.m2 = 1;
sys.j1 = 1/12;
sys.j2 = 1/12;
sys.r1=1/2;
sys.r2=1/2;
M1=sys.m1*sys.r1^2+sys.m2*sys.l1^2+sys.j1;
M2=sys.m2*sys.r2^2+sys.j2;
M3=sys.m2*sys.l1*sys.r2;
sys.M=@(q) [M1+M2+2*M3*cos(q(2)) M2+M3*cos(q(2));
           M2+M3*cos(q(2)) M2];
        
 % Ignore the friction.
sys.D= @(q) diag([0.5,0.5]);

sys.G = @(q) [1 0; 0 1];
% Potential energy parameters
sys.g = 9.8;
sys.V = @(q) sys.g*sys.m1*sys.r1*sin(q(1)) + (sys.g*sys.m2*(sys.l1*sin(q(1))+sys.r2*sin(q(1)+q(2))));

% Define open-loop system energy
sys.H = @(q,p) 0.5*p.'*(sys.M(q)\p) + sys.V(q);
sys.T = @(q,p) 0.5*p.'*(sys.M(q)\p);

% Define open-loop energy gradients
sys.dVdq = matlabFunction(jacobian(sys.V(q_sym),q_sym).','vars',{q_sym});

sys.Ddq = matlabFunction(jacobian(sys.H(q_sym,p_sym),q_sym).','vars',[{q_sym}, {p_sym}]);
sys.Ddp =  matlabFunction(jacobian(sys.H(q_sym,p_sym),p_sym).','vars',[{q_sym}, {p_sym}]);
% Define system ODE Port-Hmailton Model of 2dof's robotic arm in eq.(25)
%2DOF机械臂的port-hamilton model
dx = @(q,p,u,t) [zeros(2) eye(2); -eye(2) -sys.D(q)]*[sys.Ddq(q,p); sys.Ddp(q,p)] + [zeros(2); sys.G(q)]*u;

%% Control law
% Compute the matrix T(q).
ctrl.T =matlabFunction(manualCholesky(inv(sys.M(q_sym))),'vars',{q_sym});

ctrl.p = @(q,p) ctrl.T(q)'*p;
ctrl.D = @(q) sys.D(q);
ctrl.G = @(q) ctrl.T(q)'*sys.G(q);
% Define target trajectory and derivatives
%定义轨迹
% ctrl.qd = @(t) [pi/2 + (1/2)*cos(t); (1/2)*sin(t)];
% ctrl.dqd = @(t) [-(1/2)*sin(t); (1/2)*cos(t)];
% ctrl.ddqd = @(t) [-(1/2)*cos(t); -(1/2)*sin(t)];
  ctrl.qd = @(t) [1;1];
ctrl.dqd = @(t) [0;0];
ctrl.ddqd = @(t) [0;0];
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

phi = @(q,p) 10*q+p;%Here q is a variable to be determined
%phi=@(q,p) (K*q)+tan(p);%Here q is a variable to be determined
%Take a partial derivative of \phi
dphidq = matlabFunction(jacobian(phi(q_sym,p_sym),q_sym),'vars',{q_sym});
dphidp = matlabFunction(jacobian(phi(q_sym,p_sym),p_sym),'vars',{p_sym});
dphideq =dphidq;
dphidep =dphidp;
%Replace the variable to be determined q with eq(\tilde \q)
% dphideq = matlabFunction(dphidq(ctrl.eq(t_sym,q_sym)),'vars',[{t_sym},{q_sym}]);
% dphidep=matlabFunction(dphidp(ctrl.ep(t_sym,q_sym,p_sym)),'vars',[{t_sym},{q_sym},{p_sym}]);
%compute the Lambda from eq.(22)
 % he = 0.5*(dphideq(t_sym,q_sym)*ctrl.T(q_sym))*dphidp()';%这里的系数对系统收敛到滑模面上有影响
  Kd=[1,0;0,4];
 he =@(q) 0.5*dphideq(q)*(ctrl.D(q)+Kd)*dphidp()';%这里的系数对系统收敛到滑模面上有影响

Lambda = matlabFunction(2*(he(q_sym)+he(q_sym)'),'vars',q_sym);
%z =@(q,p) phi(ctrl.eq(0,sim.q0),ctrl.ep(0,sim.q0,ctrl.p(sim.q0,sim.p0))) + Lambda(q)*dUdPhi(t,q,p)
%int(Lambda(t_sym,q_sym,p_sym)*dUdPhi(t_sym,q_sym,p_sym),t_sym)
%Compute the partial derivative of U
normPhi=phi(ctrl.eq(t_sym,q_sym),ctrl.ep(t_sym,q_sym,p_sym));
dUdPhi=matlabFunction((phi(ctrl.eq(t_sym,q_sym),ctrl.ep(t_sym,q_sym,p_sym))/(sqrt(sum(normPhi.^2)))),'vars',[{t_sym},{q_sym},{p_sym}]);

%Then the feedback controller from eq.(23)
epslion=-0;
ctrl.v = @(t,q,p) epslion*(ctrl.ep(t,q,p)+dphidp(p)'*dUdPhi(t,q,p))+(-Lambda(q)*dUdPhi(t,q,p)+(ctrl.D(q))*ctrl.ep(t,q,p)-inv(dphidp())*dphideq(q)*(ctrl.D(q)+Kd)*ctrl.ep(t,q,p));
%ctrl.v = @(t,q,p) (-inv(dphidep(t,q,p))*Lambda(t,q,p)*dUdPhi(t,q,p)+(ctrl.D(q))*ctrl.ep(t,q,p)-((inv(dphidep(t,q,p))*dphidq())*ctrl.T(q))*ctrl.ep(t,q,p));

%input u from eq.(20)
ctrl.u = @(t,q,p) ctrl.G(q)\(ctrl.D(q)*ctrl.pd(t,q) + ctrl.dpddq(t,q)*(ctrl.T(q)*p) + ctrl.T(q)\ctrl.ddqd(t) + ctrl.T(q)'*sys.dVdq(q) + ctrl.v(t,q,p));
%ctrl.u = @(t,q,p) ctrl.G(q)\(epslion*(p+6*q)+ ctrl.T(q)'*sys.dVdq(q));
% Define  closed-loop energy in eq.(24)
ctrl.KE = @(t,q,p) 0.5*sum(ctrl.ep(t,q,p).^2);%kinetic energy
ctrl.U=matlabFunction((sqrt(sum(normPhi.^2))),'vars',[{t_sym},{q_sym},{p_sym}]);%potential energy
ctrl.Hsmc = @(t,q,p) ctrl.KE(t,q,p) + ctrl.U(t,q,p);
%% Run simulation


% Comcatinate model with control law
ode = @(t,x) [dx(x(1:2),x(3:4),ctrl.u(t,x(1:2),ctrl.p(x(1:2),x(3:4))),t)];
% Solve ODE
options = odeset('OutputFcn', @myOutputFcn,'RelTol',0.3e-1);
[res.t,res.x] = ode78(ode,[0:sim.delta_t:sim.t_end],sim.x0,options);
%% Plot output
% Unpack solution vector. Solution is in cannonical coordinates
res.q = res.x(:,1:2);
res.p0 = res.x(:,3:4);

% Compute qualtities of interest
res.H = zeros(length(res.t),1);
res.Hsmc = zeros(length(res.t),1);
res.Hcl = zeros(length(res.t),1);
res.Hp = zeros(length(res.t),1);
res.qd = zeros(length(res.t),2);
res.qe = zeros(length(res.t),2);
res.phi=zeros(length(res.t),2);
for i=1:length(res.t)
    res.H(i)    = sys.H(res.q(i,:).',res.p0(i,:).');
    res.p(i,:)  = (ctrl.T(res.q(i,:).')'*res.p0(i,:).').';
    res.Hsmc(i) = ctrl.Hsmc(res.t(i),res.q(i,:).',res.p(i,:).');
    res.qd(i,:) = ctrl.qd(res.t(i)).';
    res.pd(i,:) =ctrl.pd(res.t(i),ctrl.qd(res.t(i))).';
    res.qe(i,:) = res.q(i,:) - res.qd(i,:);
    res.pe(i,:) = res.p(i,:) - res.pd(i,:);
    res.phi(i,:)=phi(res.qe(i,:).',res.pe(i,:).');
    res.u(i,:)  =ctrl.u(res.t(i,:).',res.q(i,:).',ctrl.p(res.q(i,:).',res.p0(i,:).'));
    
end

% plot energy
fig1 = figure(1)
subplot(2,1,1)
plot(res.t,res.H)
xlabel('time (s)')
ylabel('Open-loop energy')
subplot(2,1,2)

plot(res.t,res.Hsmc)
grid on
xlabel('time (s)')
ylabel('Log of closed-loop energy $H_{smc}$')

% plot configuration
fig2 = figure(2)
plot(res.t,res.q,res.t,res.qd,'--')
legend('$q_1$','$q_2$')
xlabel('time (s)')
ylabel('Configuration')
title('The responses of q')
grid on
fig3=figure(3)
plot(res.t,res.phi)
title('The responses of $\sigma$')
fig4=figure(4)
plot(res.t,res.u)
title('The responses of u')
fig5=figure(5)
plot(res.t,res.qe)
title('The responses of error')

%% 
function [T]= manualCholesky(A)
% 输入：A为一个n x n的实对称正定矩阵
% 输出：L为A的Cholesky分解得到的下三角矩阵
%Input:A is an n x n real symmetric positive definite matrix
%Output:T is the lower triangular matrix obtained by the Cholesky decomposition of A

n = size(A, 1); % 获取矩阵的维度
L = sym(zeros(n)); % 初始化L为n x n的零矩阵

% Cholesky分解的实现
for i = 1:n
    for k = 1:i
        sum = 0;
        for j = 1:(k-1)
            sum = sum + L(i,j) * L(k,j);
        end
        
        if i == k % 对角线元素
            L(i,i) = sqrt(A(i,i) - sum); % 计算对角线元素
        else
            L(i,k) = (A(i,k) - sum) / L(k,k); % 计算非对角线元素
        end
    end
end
T=L;
end
% 迭代输出函数
function status = myOutputFcn(t, y, flag)
    if strcmp(flag, 'init')
        fprintf('Simulation started.\n');
    elseif strcmp(flag, '')
        fprintf('At time %.2f \n', t);
    elseif strcmp(flag, 'done')
        fprintf('Simulation finished.\n');
    end
    status = [];
end